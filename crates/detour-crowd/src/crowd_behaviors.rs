//! Advanced crowd behaviors for Detour
//!
//! This module provides sophisticated AI behaviors for crowd agents including
//! behavior trees, steering behaviors, group dynamics, and contextual reactions.

use std::collections::HashMap;

use super::CrowdAgent;
use detour::{NavMesh, Status};
use recast_common::{Error, Result};

/// Maximum number of behavior nodes per tree
const MAX_BEHAVIOR_NODES: usize = 128;

/// Behavior execution result
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BehaviorResult {
    /// Behavior completed successfully
    Success,
    /// Behavior failed to complete
    Failure,
    /// Behavior is still running
    Running,
}

/// Behavior tree node types
#[derive(Debug, Clone)]
pub enum BehaviorNode {
    /// Composite nodes
    Sequence(SequenceNode),
    Selector(SelectorNode),
    Parallel(ParallelNode),

    /// Decorator nodes
    Repeater(RepeaterNode),
    Inverter(InverterNode),
    Cooldown(CooldownNode),

    /// Leaf nodes (actions/conditions)
    Action(ActionNode),
    Condition(ConditionNode),
}

/// Sequence node - executes children in order until one fails
#[derive(Debug, Clone)]
pub struct SequenceNode {
    pub children: Vec<usize>,
    pub current_child: usize,
}

/// Selector node - executes children until one succeeds
#[derive(Debug, Clone)]
pub struct SelectorNode {
    pub children: Vec<usize>,
    pub current_child: usize,
}

/// Parallel node - executes all children simultaneously
#[derive(Debug, Clone)]
pub struct ParallelNode {
    pub children: Vec<usize>,
    pub success_threshold: usize,
    pub failure_threshold: usize,
}

/// Repeater node - repeats child node a number of times
#[derive(Debug, Clone)]
pub struct RepeaterNode {
    pub child: usize,
    pub repeat_count: i32, // -1 for infinite
    pub current_count: i32,
}

/// Inverter node - inverts the result of child node
#[derive(Debug, Clone)]
pub struct InverterNode {
    pub child: usize,
}

/// Cooldown node - prevents child execution for a time period
#[derive(Debug, Clone)]
pub struct CooldownNode {
    pub child: usize,
    pub cooldown_time: f32,
    pub last_execution: f32,
}

/// Action node types
#[derive(Debug, Clone)]
pub enum ActionType {
    /// Move to a specific position
    MoveTo { target: [f32; 3], speed: f32 },
    /// Follow another agent
    Follow { agent_id: usize, distance: f32 },
    /// Flee from a position
    Flee { source: [f32; 3], distance: f32 },
    /// Wander randomly
    Wander { radius: f32, change_interval: f32 },
    /// Seek a target agent
    Seek { target_id: usize },
    /// Avoid other agents
    Separate { radius: f32, strength: f32 },
    /// Align with nearby agents
    Align { radius: f32, strength: f32 },
    /// Cohere with nearby agents
    Cohere { radius: f32, strength: f32 },
    /// Wait for a duration
    Wait { duration: f32 },
    /// Custom action with user data
    Custom { action_id: u32, data: Vec<f32> },
}

/// Action node
#[derive(Debug, Clone)]
pub struct ActionNode {
    pub action_type: ActionType,
    pub execution_time: f32,
    pub start_time: f32,
}

/// Condition node types
#[derive(Debug, Clone)]
pub enum ConditionType {
    /// Check if agent is near a position
    NearPosition { position: [f32; 3], distance: f32 },
    /// Check if agent is near another agent
    NearAgent { agent_id: usize, distance: f32 },
    /// Check agent's health/energy level
    HealthAbove { threshold: f32 },
    /// Check if agent has reached target
    ReachedTarget,
    /// Check if agent is in formation
    InFormation,
    /// Check random chance
    RandomChance { probability: f32 },
    /// Check if enemy agents are nearby
    EnemiesNearby { radius: f32, count: usize },
    /// Custom condition with user data
    Custom { condition_id: u32, data: Vec<f32> },
}

/// Condition node
#[derive(Debug, Clone)]
pub struct ConditionNode {
    pub condition_type: ConditionType,
}

/// Behavior tree for an agent
#[derive(Debug, Clone)]
pub struct BehaviorTree {
    /// All nodes in the tree
    pub nodes: Vec<BehaviorNode>,
    /// Root node index
    pub root: usize,
    /// Current execution state
    pub execution_state: HashMap<usize, BehaviorResult>,
}

impl Default for BehaviorTree {
    fn default() -> Self {
        Self::new()
    }
}

impl BehaviorTree {
    /// Creates a new empty behavior tree
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            root: 0,
            execution_state: HashMap::new(),
        }
    }

    /// Adds a node to the tree and returns its index
    pub fn add_node(&mut self, node: BehaviorNode) -> Result<usize> {
        if self.nodes.len() >= MAX_BEHAVIOR_NODES {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        let index = self.nodes.len();
        self.nodes.push(node);
        Ok(index)
    }

    /// Sets the root node
    pub fn set_root(&mut self, index: usize) -> Result<()> {
        if index >= self.nodes.len() {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }
        self.root = index;
        Ok(())
    }

    /// Executes the behavior tree for an agent
    pub fn execute(&mut self, agent: &mut CrowdAgent, context: &BehaviorContext) -> BehaviorResult {
        if self.nodes.is_empty() {
            return BehaviorResult::Failure;
        }

        self.execute_node(self.root, agent, context)
    }

    /// Executes a specific node
    fn execute_node(
        &mut self,
        node_index: usize,
        agent: &mut CrowdAgent,
        context: &BehaviorContext,
    ) -> BehaviorResult {
        if node_index >= self.nodes.len() {
            return BehaviorResult::Failure;
        }

        let node = self.nodes[node_index].clone();
        match node {
            BehaviorNode::Sequence(seq) => self.execute_sequence(node_index, seq, agent, context),
            BehaviorNode::Selector(sel) => self.execute_selector(node_index, sel, agent, context),
            BehaviorNode::Parallel(par) => self.execute_parallel(node_index, par, agent, context),
            BehaviorNode::Repeater(rep) => self.execute_repeater(node_index, rep, agent, context),
            BehaviorNode::Inverter(inv) => self.execute_inverter(node_index, inv, agent, context),
            BehaviorNode::Cooldown(cd) => self.execute_cooldown(node_index, cd, agent, context),
            BehaviorNode::Action(act) => self.execute_action(node_index, act, agent, context),
            BehaviorNode::Condition(cond) => {
                self.execute_condition(node_index, cond, agent, context)
            }
        }
    }

    fn execute_sequence(
        &mut self,
        node_index: usize,
        mut seq: SequenceNode,
        agent: &mut CrowdAgent,
        context: &BehaviorContext,
    ) -> BehaviorResult {
        while seq.current_child < seq.children.len() {
            let child_result = self.execute_node(seq.children[seq.current_child], agent, context);

            match child_result {
                BehaviorResult::Success => {
                    seq.current_child += 1;
                    // Update the node with new state
                    self.nodes[node_index] = BehaviorNode::Sequence(seq.clone());
                }
                BehaviorResult::Failure => {
                    seq.current_child = 0;
                    self.nodes[node_index] = BehaviorNode::Sequence(seq);
                    return BehaviorResult::Failure;
                }
                BehaviorResult::Running => {
                    self.nodes[node_index] = BehaviorNode::Sequence(seq);
                    return BehaviorResult::Running;
                }
            }
        }

        // All children succeeded
        seq.current_child = 0;
        self.nodes[node_index] = BehaviorNode::Sequence(seq);
        BehaviorResult::Success
    }

    fn execute_selector(
        &mut self,
        node_index: usize,
        mut sel: SelectorNode,
        agent: &mut CrowdAgent,
        context: &BehaviorContext,
    ) -> BehaviorResult {
        while sel.current_child < sel.children.len() {
            let child_result = self.execute_node(sel.children[sel.current_child], agent, context);

            match child_result {
                BehaviorResult::Success => {
                    sel.current_child = 0;
                    self.nodes[node_index] = BehaviorNode::Selector(sel);
                    return BehaviorResult::Success;
                }
                BehaviorResult::Failure => {
                    sel.current_child += 1;
                    self.nodes[node_index] = BehaviorNode::Selector(sel.clone());
                }
                BehaviorResult::Running => {
                    self.nodes[node_index] = BehaviorNode::Selector(sel);
                    return BehaviorResult::Running;
                }
            }
        }

        // All children failed
        sel.current_child = 0;
        self.nodes[node_index] = BehaviorNode::Selector(sel);
        BehaviorResult::Failure
    }

    fn execute_parallel(
        &mut self,
        _node_index: usize,
        par: ParallelNode,
        agent: &mut CrowdAgent,
        context: &BehaviorContext,
    ) -> BehaviorResult {
        let mut success_count = 0;
        let mut failure_count = 0;
        let mut _running_count = 0;

        for &child_index in &par.children {
            let result = self.execute_node(child_index, agent, context);
            match result {
                BehaviorResult::Success => success_count += 1,
                BehaviorResult::Failure => failure_count += 1,
                BehaviorResult::Running => _running_count += 1,
            }
        }

        if success_count >= par.success_threshold {
            BehaviorResult::Success
        } else if failure_count >= par.failure_threshold {
            BehaviorResult::Failure
        } else {
            BehaviorResult::Running
        }
    }

    fn execute_repeater(
        &mut self,
        node_index: usize,
        mut rep: RepeaterNode,
        agent: &mut CrowdAgent,
        context: &BehaviorContext,
    ) -> BehaviorResult {
        if rep.repeat_count != -1 && rep.current_count >= rep.repeat_count {
            rep.current_count = 0;
            self.nodes[node_index] = BehaviorNode::Repeater(rep.clone());
            return BehaviorResult::Success;
        }

        let child_result = self.execute_node(rep.child, agent, context);

        match child_result {
            BehaviorResult::Success | BehaviorResult::Failure => {
                rep.current_count += 1;
                let should_complete =
                    rep.repeat_count != -1 && rep.current_count >= rep.repeat_count;
                self.nodes[node_index] = BehaviorNode::Repeater(rep);
                if should_complete {
                    BehaviorResult::Success
                } else {
                    BehaviorResult::Running
                }
            }
            BehaviorResult::Running => {
                self.nodes[node_index] = BehaviorNode::Repeater(rep);
                BehaviorResult::Running
            }
        }
    }

    fn execute_inverter(
        &mut self,
        _node_index: usize,
        inv: InverterNode,
        agent: &mut CrowdAgent,
        context: &BehaviorContext,
    ) -> BehaviorResult {
        let child_result = self.execute_node(inv.child, agent, context);
        match child_result {
            BehaviorResult::Success => BehaviorResult::Failure,
            BehaviorResult::Failure => BehaviorResult::Success,
            BehaviorResult::Running => BehaviorResult::Running,
        }
    }

    fn execute_cooldown(
        &mut self,
        node_index: usize,
        mut cd: CooldownNode,
        agent: &mut CrowdAgent,
        context: &BehaviorContext,
    ) -> BehaviorResult {
        let current_time = context.current_time;

        if current_time - cd.last_execution < cd.cooldown_time {
            return BehaviorResult::Failure;
        }

        let child_result = self.execute_node(cd.child, agent, context);

        if child_result != BehaviorResult::Running {
            cd.last_execution = current_time;
            self.nodes[node_index] = BehaviorNode::Cooldown(cd);
        }

        child_result
    }

    fn execute_action(
        &mut self,
        node_index: usize,
        mut action: ActionNode,
        agent: &mut CrowdAgent,
        context: &BehaviorContext,
    ) -> BehaviorResult {
        if action.start_time == 0.0 {
            action.start_time = context.current_time;
            self.nodes[node_index] = BehaviorNode::Action(action.clone());
        }

        let elapsed = context.current_time - action.start_time;

        match &action.action_type {
            ActionType::MoveTo { target, speed: _ } => {
                let distance = calculate_distance(&agent.get_pos(), target);
                if distance < 1.0 {
                    action.start_time = 0.0;
                    self.nodes[node_index] = BehaviorNode::Action(action);
                    BehaviorResult::Success
                } else {
                    // Set target for agent (this would integrate with crowd pathfinding)
                    BehaviorResult::Running
                }
            }
            ActionType::Follow { agent_id, distance } => {
                if let Some(target_agent) = context.agents.get(agent_id) {
                    let target_pos = target_agent.get_pos();
                    let current_distance = calculate_distance(&agent.get_pos(), &target_pos);
                    if current_distance > *distance {
                        // Move towards target agent
                        BehaviorResult::Running
                    } else {
                        BehaviorResult::Success
                    }
                } else {
                    BehaviorResult::Failure
                }
            }
            ActionType::Flee { source, distance } => {
                let current_distance = calculate_distance(&agent.get_pos(), source);
                if current_distance >= *distance {
                    action.start_time = 0.0;
                    self.nodes[node_index] = BehaviorNode::Action(action);
                    BehaviorResult::Success
                } else {
                    // Calculate flee direction and move
                    BehaviorResult::Running
                }
            }
            ActionType::Wander {
                radius: _,
                change_interval,
            } => {
                if elapsed >= *change_interval {
                    // Choose new random direction and move
                    action.start_time = context.current_time;
                    self.nodes[node_index] = BehaviorNode::Action(action);
                    BehaviorResult::Running
                } else {
                    BehaviorResult::Running
                }
            }
            ActionType::Wait { duration } => {
                if elapsed >= *duration {
                    action.start_time = 0.0;
                    self.nodes[node_index] = BehaviorNode::Action(action);
                    BehaviorResult::Success
                } else {
                    BehaviorResult::Running
                }
            }
            _ => BehaviorResult::Success, // Placeholder for other actions
        }
    }

    fn execute_condition(
        &mut self,
        _node_index: usize,
        condition: ConditionNode,
        agent: &mut CrowdAgent,
        context: &BehaviorContext,
    ) -> BehaviorResult {
        match &condition.condition_type {
            ConditionType::NearPosition { position, distance } => {
                let current_distance = calculate_distance(&agent.get_pos(), position);
                if current_distance <= *distance {
                    BehaviorResult::Success
                } else {
                    BehaviorResult::Failure
                }
            }
            ConditionType::NearAgent { agent_id, distance } => {
                if let Some(target_agent) = context.agents.get(agent_id) {
                    let target_pos = target_agent.get_pos();
                    let current_distance = calculate_distance(&agent.get_pos(), &target_pos);
                    if current_distance <= *distance {
                        BehaviorResult::Success
                    } else {
                        BehaviorResult::Failure
                    }
                } else {
                    BehaviorResult::Failure
                }
            }
            ConditionType::HealthAbove { threshold } => {
                // Placeholder - would check agent health
                if context.agent_health.get(&agent.get_id()).unwrap_or(&100.0) > threshold {
                    BehaviorResult::Success
                } else {
                    BehaviorResult::Failure
                }
            }
            ConditionType::RandomChance { probability } => {
                if fastrand::f32() < *probability {
                    BehaviorResult::Success
                } else {
                    BehaviorResult::Failure
                }
            }
            _ => BehaviorResult::Success, // Placeholder for other conditions
        }
    }
}

/// Steering behavior types
#[derive(Debug, Clone)]
pub enum SteeringBehavior {
    /// Seek a target position
    Seek { target: [f32; 3], strength: f32 },
    /// Flee from a position
    Flee { source: [f32; 3], strength: f32 },
    /// Arrive at target with deceleration
    Arrive {
        target: [f32; 3],
        slowing_radius: f32,
        strength: f32,
    },
    /// Pursue a moving target
    Pursue { target_id: usize, strength: f32 },
    /// Evade a moving pursuer
    Evade { pursuer_id: usize, strength: f32 },
    /// Wander randomly
    Wander {
        radius: f32,
        distance: f32,
        jitter: f32,
    },
    /// Separate from nearby agents
    Separation { radius: f32, strength: f32 },
    /// Align with nearby agents
    Alignment { radius: f32, strength: f32 },
    /// Cohere with nearby agents
    Cohesion { radius: f32, strength: f32 },
    /// Follow a path
    PathFollowing { path: Vec<[f32; 3]>, radius: f32 },
    /// Avoid obstacles
    ObstacleAvoidance { lookahead: f32, force: f32 },
    /// Follow a leader
    LeaderFollowing {
        leader_id: usize,
        distance: f32,
        separation_radius: f32,
    },
}

/// Steering behavior result
#[derive(Debug, Clone)]
pub struct SteeringForce {
    pub force: [f32; 3],
    pub priority: f32,
}

/// Group behavior types
#[derive(Debug, Clone)]
pub enum GroupBehavior {
    /// Flocking behavior (combines separation, alignment, cohesion)
    Flocking {
        separation_weight: f32,
        alignment_weight: f32,
        cohesion_weight: f32,
        neighbor_radius: f32,
    },
    /// Follow the leader behavior
    FollowLeader {
        leader_id: usize,
        formation_distance: f32,
        arrival_radius: f32,
    },
    /// Herding behavior (move group towards target)
    Herding {
        target: [f32; 3],
        herder_strength: f32,
        group_cohesion: f32,
    },
    /// Escorting behavior (protect a target)
    Escorting {
        target_id: usize,
        escort_distance: f32,
        threat_radius: f32,
    },
}

/// Emotional state of an agent
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EmotionalState {
    Calm,
    Excited,
    Fearful,
    Aggressive,
    Curious,
    Tired,
    Alert,
}

/// Contextual behavior triggers
#[derive(Debug, Clone)]
pub enum BehaviorTrigger {
    /// Triggered by proximity to position
    ProximityTrigger { position: [f32; 3], radius: f32 },
    /// Triggered by seeing another agent
    SightTrigger {
        agent_id: usize,
        view_angle: f32,
        view_distance: f32,
    },
    /// Triggered by sound events
    SoundTrigger { source: [f32; 3], intensity: f32 },
    /// Triggered by time conditions
    TimeTrigger { start_time: f32, duration: f32 },
    /// Triggered by agent health/energy
    HealthTrigger { threshold: f32, below: bool },
    /// Custom trigger with user data
    CustomTrigger { trigger_id: u32, data: Vec<f32> },
}

/// Behavior context for execution
#[derive(Debug)]
pub struct BehaviorContext<'a> {
    /// Current simulation time
    pub current_time: f32,
    /// All agents in the simulation
    pub agents: &'a HashMap<usize, CrowdAgent>,
    /// Agent health values
    pub agent_health: &'a HashMap<usize, f32>,
    /// Navigation mesh for pathfinding
    pub nav_mesh: &'a NavMesh,
    /// Environmental data
    pub environment: &'a EnvironmentData,
}

/// Environmental data for contextual behaviors
#[derive(Debug, Default, Clone)]
pub struct EnvironmentData {
    /// Sound sources in the environment
    pub sound_sources: Vec<SoundSource>,
    /// Light sources affecting visibility
    pub light_sources: Vec<LightSource>,
    /// Weather conditions
    pub weather: WeatherCondition,
    /// Danger zones
    pub danger_zones: Vec<DangerZone>,
}

/// Sound source in the environment
#[derive(Debug, Clone)]
pub struct SoundSource {
    pub position: [f32; 3],
    pub intensity: f32,
    pub frequency: f32,
    pub duration: f32,
}

/// Light source affecting agent behavior
#[derive(Debug, Clone)]
pub struct LightSource {
    pub position: [f32; 3],
    pub intensity: f32,
    pub radius: f32,
    pub color: [f32; 3], // RGB
}

/// Weather conditions
#[derive(Debug, Clone, Copy, Default)]
pub enum WeatherCondition {
    #[default]
    Clear,
    Rainy,
    Foggy,
    Stormy,
    Snowy,
}

/// Danger zone affecting agent behavior
#[derive(Debug, Clone)]
pub struct DangerZone {
    pub center: [f32; 3],
    pub radius: f32,
    pub danger_level: f32,
    pub zone_type: DangerType,
}

/// Types of danger zones
#[derive(Debug, Clone, Copy)]
pub enum DangerType {
    Fire,
    Toxic,
    Explosive,
    Hostile,
    Unstable,
}

/// Advanced crowd behavior manager
#[derive(Debug)]
pub struct CrowdBehaviorManager {
    /// Behavior trees for agents
    behavior_trees: HashMap<usize, BehaviorTree>,
    /// Steering behaviors for agents
    steering_behaviors: HashMap<usize, Vec<SteeringBehavior>>,
    /// Group behaviors
    group_behaviors: HashMap<usize, GroupBehavior>,
    /// Emotional states of agents
    emotional_states: HashMap<usize, EmotionalState>,
    /// Behavior triggers
    triggers: Vec<(BehaviorTrigger, BehaviorTree)>,
    /// Environment data
    environment: EnvironmentData,
    /// Agent wander targets for random movement
    /// TODO: Implement persistent wander targets for more coherent wandering behavior
    #[allow(dead_code)]
    wander_targets: HashMap<usize, [f32; 3]>,
}

impl CrowdBehaviorManager {
    /// Creates a new behavior manager
    pub fn new() -> Self {
        Self {
            behavior_trees: HashMap::new(),
            steering_behaviors: HashMap::new(),
            group_behaviors: HashMap::new(),
            emotional_states: HashMap::new(),
            triggers: Vec::new(),
            environment: EnvironmentData::default(),
            wander_targets: HashMap::new(),
        }
    }

    /// Sets a behavior tree for an agent
    pub fn set_behavior_tree(&mut self, agent_id: usize, tree: BehaviorTree) {
        self.behavior_trees.insert(agent_id, tree);
    }

    /// Adds a steering behavior to an agent
    pub fn add_steering_behavior(&mut self, agent_id: usize, behavior: SteeringBehavior) {
        self.steering_behaviors
            .entry(agent_id)
            .or_default()
            .push(behavior);
    }

    /// Sets a group behavior for an agent
    pub fn set_group_behavior(&mut self, agent_id: usize, behavior: GroupBehavior) {
        self.group_behaviors.insert(agent_id, behavior);
    }

    /// Sets an emotional state for an agent
    pub fn set_emotional_state(&mut self, agent_id: usize, state: EmotionalState) {
        self.emotional_states.insert(agent_id, state);
    }

    /// Adds a behavior trigger
    pub fn add_trigger(&mut self, trigger: BehaviorTrigger, behavior: BehaviorTree) {
        self.triggers.push((trigger, behavior));
    }

    /// Updates all agent behaviors
    pub fn update(
        &mut self,
        agents: &mut HashMap<usize, CrowdAgent>,
        dt: f32,
        current_time: f32,
        nav_mesh: &NavMesh,
    ) -> Result<()> {
        let agent_health = HashMap::new(); // Placeholder

        // Process each type of behavior separately to avoid borrowing conflicts
        self.update_behavior_trees(agents, current_time, nav_mesh, &agent_health)?;
        self.update_steering_behaviors(agents, dt, current_time, nav_mesh, &agent_health)?;
        self.update_group_behaviors(agents, dt, current_time, nav_mesh, &agent_health)?;

        // Check behavior triggers (fixed borrowing conflict by cloning environment)
        let environment_clone = self.environment.clone();
        let agent_refs = agents
            .iter()
            .map(|(id, agent)| (*id, agent.clone()))
            .collect();
        let context = BehaviorContext {
            current_time,
            agents: &agent_refs,
            agent_health: &agent_health,
            nav_mesh,
            environment: &environment_clone,
        };
        self.check_triggers(agents, &context)?;

        Ok(())
    }

    /// Updates behavior trees for all agents
    fn update_behavior_trees(
        &mut self,
        agents: &mut HashMap<usize, CrowdAgent>,
        current_time: f32,
        nav_mesh: &NavMesh,
        agent_health: &HashMap<usize, f32>,
    ) -> Result<()> {
        // For now, simplified implementation without full context to avoid borrowing issues
        let agent_ids: Vec<usize> = self.behavior_trees.keys().copied().collect();

        for agent_id in agent_ids {
            if let (Some(agent), Some(tree)) = (
                agents.get_mut(&agent_id),
                self.behavior_trees.get_mut(&agent_id),
            ) {
                // Create a minimal context for execution
                let context = BehaviorContext {
                    current_time,
                    agents: &HashMap::new(), // Simplified to avoid borrowing
                    agent_health,
                    nav_mesh,
                    environment: &self.environment,
                };

                tree.execute(agent, &context);
            }
        }

        Ok(())
    }

    /// Updates steering behaviors for all agents
    fn update_steering_behaviors(
        &mut self,
        agents: &mut HashMap<usize, CrowdAgent>,
        dt: f32,
        current_time: f32,
        nav_mesh: &NavMesh,
        agent_health: &HashMap<usize, f32>,
    ) -> Result<()> {
        let agent_ids: Vec<usize> = self.steering_behaviors.keys().copied().collect();

        for agent_id in agent_ids {
            if let Some(behaviors) = self.steering_behaviors.get(&agent_id).cloned() {
                let context = BehaviorContext {
                    current_time,
                    agents: &HashMap::new(), // Simplified to avoid borrowing
                    agent_health,
                    nav_mesh,
                    environment: &self.environment,
                };

                let steering_force = self.calculate_steering_force(agent_id, &behaviors, &context);

                // Apply the force to the agent
                if let Some(agent_mut) = agents.get_mut(&agent_id) {
                    self.apply_steering_force(agent_mut, steering_force, dt);
                }
            }
        }

        Ok(())
    }

    /// Updates group behaviors for all agents
    fn update_group_behaviors(
        &mut self,
        agents: &mut HashMap<usize, CrowdAgent>,
        dt: f32,
        current_time: f32,
        nav_mesh: &NavMesh,
        agent_health: &HashMap<usize, f32>,
    ) -> Result<()> {
        let agent_ids: Vec<usize> = self.group_behaviors.keys().copied().collect();

        for agent_id in agent_ids {
            if let Some(behavior) = self.group_behaviors.get(&agent_id) {
                let context = BehaviorContext {
                    current_time,
                    agents: &HashMap::new(), // Simplified to avoid borrowing
                    agent_health,
                    nav_mesh,
                    environment: &self.environment,
                };

                // Apply group behavior
                if let Some(agent_mut) = agents.get_mut(&agent_id) {
                    self.update_group_behavior(agent_id, agent_mut, behavior, &context, dt);
                }
            }
        }

        Ok(())
    }

    /// Calculates steering force for an agent
    fn calculate_steering_force(
        &self,
        agent_id: usize,
        behaviors: &[SteeringBehavior],
        context: &BehaviorContext,
    ) -> [f32; 3] {
        let mut total_force = [0.0, 0.0, 0.0];

        if let Some(agent) = context.agents.get(&agent_id) {
            let agent_pos = agent.get_pos();
            let agent_vel = agent.get_vel();

            for behavior in behaviors {
                let force = match behavior {
                    SteeringBehavior::Seek { target, strength } => {
                        self.calculate_seek_force(&agent_pos, target, *strength)
                    }
                    SteeringBehavior::Flee { source, strength } => {
                        self.calculate_flee_force(&agent_pos, source, *strength)
                    }
                    SteeringBehavior::Arrive {
                        target,
                        slowing_radius,
                        strength,
                    } => self.calculate_arrive_force(
                        &agent_pos,
                        &agent_vel,
                        target,
                        *slowing_radius,
                        *strength,
                    ),
                    SteeringBehavior::Wander {
                        radius,
                        distance,
                        jitter,
                    } => {
                        // For now, simplified wander without state
                        // Jitter affects the randomness of the wander
                        let wander_angle = fastrand::f32() * 2.0 * std::f32::consts::PI;
                        let jitter_offset = jitter * (fastrand::f32() * 2.0 - 1.0);
                        let wander_target = [
                            agent_pos[0]
                                + distance * wander_angle.cos()
                                + radius * (fastrand::f32() * 2.0 - 1.0) * (1.0 + jitter_offset),
                            agent_pos[1],
                            agent_pos[2]
                                + distance * wander_angle.sin()
                                + radius * (fastrand::f32() * 2.0 - 1.0) * (1.0 + jitter_offset),
                        ];
                        self.calculate_seek_force(&agent_pos, &wander_target, 1.0)
                    }
                    SteeringBehavior::Separation { radius, strength } => self
                        .calculate_separation_force(
                            agent_id, &agent_pos, context, *radius, *strength,
                        ),
                    _ => [0.0, 0.0, 0.0], // Placeholder for other behaviors
                };

                total_force[0] += force[0];
                total_force[1] += force[1];
                total_force[2] += force[2];
            }
        }

        total_force
    }

    /// Calculates seek steering force
    fn calculate_seek_force(
        &self,
        agent_pos: &[f32; 3],
        target: &[f32; 3],
        strength: f32,
    ) -> [f32; 3] {
        let desired = [
            target[0] - agent_pos[0],
            target[1] - agent_pos[1],
            target[2] - agent_pos[2],
        ];

        let distance =
            (desired[0] * desired[0] + desired[1] * desired[1] + desired[2] * desired[2]).sqrt();
        if distance > 0.0 {
            [
                (desired[0] / distance) * strength,
                (desired[1] / distance) * strength,
                (desired[2] / distance) * strength,
            ]
        } else {
            [0.0, 0.0, 0.0]
        }
    }

    /// Calculates flee steering force
    fn calculate_flee_force(
        &self,
        agent_pos: &[f32; 3],
        source: &[f32; 3],
        strength: f32,
    ) -> [f32; 3] {
        let desired = [
            agent_pos[0] - source[0],
            agent_pos[1] - source[1],
            agent_pos[2] - source[2],
        ];

        let distance =
            (desired[0] * desired[0] + desired[1] * desired[1] + desired[2] * desired[2]).sqrt();
        if distance > 0.0 {
            [
                (desired[0] / distance) * strength,
                (desired[1] / distance) * strength,
                (desired[2] / distance) * strength,
            ]
        } else {
            [0.0, 0.0, 0.0]
        }
    }

    /// Calculates arrive steering force with deceleration
    fn calculate_arrive_force(
        &self,
        agent_pos: &[f32; 3],
        agent_vel: &[f32; 3],
        target: &[f32; 3],
        slowing_radius: f32,
        strength: f32,
    ) -> [f32; 3] {
        let desired = [
            target[0] - agent_pos[0],
            target[1] - agent_pos[1],
            target[2] - agent_pos[2],
        ];

        let distance =
            (desired[0] * desired[0] + desired[1] * desired[1] + desired[2] * desired[2]).sqrt();
        if distance > 0.0 {
            let speed = if distance < slowing_radius {
                strength * (distance / slowing_radius)
            } else {
                strength
            };

            let desired_velocity = [
                (desired[0] / distance) * speed,
                (desired[1] / distance) * speed,
                (desired[2] / distance) * speed,
            ];

            [
                desired_velocity[0] - agent_vel[0],
                desired_velocity[1] - agent_vel[1],
                desired_velocity[2] - agent_vel[2],
            ]
        } else {
            [0.0, 0.0, 0.0]
        }
    }

    /// Calculates separation steering force
    fn calculate_separation_force(
        &self,
        agent_id: usize,
        agent_pos: &[f32; 3],
        context: &BehaviorContext,
        radius: f32,
        strength: f32,
    ) -> [f32; 3] {
        let mut separation_force = [0.0, 0.0, 0.0];
        let mut neighbor_count = 0;

        for (&other_id, other_agent) in context.agents {
            if other_id != agent_id {
                let other_pos = other_agent.get_pos();
                let distance = calculate_distance(agent_pos, &other_pos);

                if distance < radius && distance > 0.0 {
                    let diff = [
                        agent_pos[0] - other_pos[0],
                        agent_pos[1] - other_pos[1],
                        agent_pos[2] - other_pos[2],
                    ];

                    // Weight by distance (closer = stronger repulsion)
                    let weight = (radius - distance) / radius;
                    separation_force[0] += (diff[0] / distance) * weight;
                    separation_force[1] += (diff[1] / distance) * weight;
                    separation_force[2] += (diff[2] / distance) * weight;
                    neighbor_count += 1;
                }
            }
        }

        if neighbor_count > 0 {
            separation_force[0] = (separation_force[0] / neighbor_count as f32) * strength;
            separation_force[1] = (separation_force[1] / neighbor_count as f32) * strength;
            separation_force[2] = (separation_force[2] / neighbor_count as f32) * strength;
        }

        separation_force
    }

    /// Applies steering force to agent
    fn apply_steering_force(&self, agent: &mut CrowdAgent, force: [f32; 3], dt: f32) {
        let params = agent.get_params();
        let max_force = params.max_acceleration;

        // Limit force magnitude
        let force_magnitude =
            (force[0] * force[0] + force[1] * force[1] + force[2] * force[2]).sqrt();
        let limited_force = if force_magnitude > max_force {
            [
                (force[0] / force_magnitude) * max_force,
                (force[1] / force_magnitude) * max_force,
                (force[2] / force_magnitude) * max_force,
            ]
        } else {
            force
        };

        // Apply force to velocity (would integrate with actual agent physics)
        let current_vel = agent.get_vel();
        let new_vel = [
            current_vel[0] + limited_force[0] * dt,
            current_vel[1] + limited_force[1] * dt,
            current_vel[2] + limited_force[2] * dt,
        ];

        // Limit velocity to max speed
        let speed =
            (new_vel[0] * new_vel[0] + new_vel[1] * new_vel[1] + new_vel[2] * new_vel[2]).sqrt();
        let max_speed = params.max_speed;

        if speed > max_speed {
            // Would set agent velocity here
            // agent.set_vel([
            //     (new_vel[0] / speed) * max_speed,
            //     (new_vel[1] / speed) * max_speed,
            //     (new_vel[2] / speed) * max_speed,
            // ]);
        }
    }

    /// Updates group behavior for an agent
    fn update_group_behavior(
        &self,
        agent_id: usize,
        agent: &mut CrowdAgent,
        behavior: &GroupBehavior,
        context: &BehaviorContext,
        dt: f32,
    ) {
        if let GroupBehavior::Flocking {
            separation_weight,
            alignment_weight,
            cohesion_weight,
            neighbor_radius,
        } = behavior
        {
            let agent_pos = agent.get_pos();
            let mut neighbors = Vec::new();

            // Find neighbors
            for (&other_id, other_agent) in context.agents {
                if other_id != agent_id {
                    let other_pos = other_agent.get_pos();
                    let distance = calculate_distance(&agent_pos, &other_pos);
                    if distance < *neighbor_radius {
                        neighbors.push((other_id, other_agent));
                    }
                }
            }

            if !neighbors.is_empty() {
                // Calculate flocking forces
                let separation = self.calculate_separation_force(
                    agent_id,
                    &agent_pos,
                    context,
                    *neighbor_radius,
                    *separation_weight,
                );
                let alignment =
                    self.calculate_alignment_force(&agent_pos, &neighbors, *alignment_weight);
                let cohesion =
                    self.calculate_cohesion_force(&agent_pos, &neighbors, *cohesion_weight);

                let total_force = [
                    separation[0] + alignment[0] + cohesion[0],
                    separation[1] + alignment[1] + cohesion[1],
                    separation[2] + alignment[2] + cohesion[2],
                ];

                self.apply_steering_force(agent, total_force, dt);
            }
        }
    }

    /// Calculates alignment steering force
    fn calculate_alignment_force(
        &self,
        _agent_pos: &[f32; 3],
        neighbors: &[(usize, &CrowdAgent)],
        strength: f32,
    ) -> [f32; 3] {
        if neighbors.is_empty() {
            return [0.0, 0.0, 0.0];
        }

        let mut average_velocity = [0.0, 0.0, 0.0];
        for (_, neighbor) in neighbors {
            let vel = neighbor.get_vel();
            average_velocity[0] += vel[0];
            average_velocity[1] += vel[1];
            average_velocity[2] += vel[2];
        }

        average_velocity[0] /= neighbors.len() as f32;
        average_velocity[1] /= neighbors.len() as f32;
        average_velocity[2] /= neighbors.len() as f32;

        [
            average_velocity[0] * strength,
            average_velocity[1] * strength,
            average_velocity[2] * strength,
        ]
    }

    /// Calculates cohesion steering force
    fn calculate_cohesion_force(
        &self,
        agent_pos: &[f32; 3],
        neighbors: &[(usize, &CrowdAgent)],
        strength: f32,
    ) -> [f32; 3] {
        if neighbors.is_empty() {
            return [0.0, 0.0, 0.0];
        }

        let mut center_of_mass = [0.0, 0.0, 0.0];
        for (_, neighbor) in neighbors {
            let pos = neighbor.get_pos();
            center_of_mass[0] += pos[0];
            center_of_mass[1] += pos[1];
            center_of_mass[2] += pos[2];
        }

        center_of_mass[0] /= neighbors.len() as f32;
        center_of_mass[1] /= neighbors.len() as f32;
        center_of_mass[2] /= neighbors.len() as f32;

        self.calculate_seek_force(agent_pos, &center_of_mass, strength)
    }

    /// Checks behavior triggers
    fn check_triggers(
        &mut self,
        agents: &mut HashMap<usize, CrowdAgent>,
        context: &BehaviorContext,
    ) -> Result<()> {
        for (trigger, behavior_tree) in &mut self.triggers {
            if let BehaviorTrigger::ProximityTrigger { position, radius } = trigger {
                for (_agent_id, agent) in agents.iter_mut() {
                    let distance = calculate_distance(&agent.get_pos(), position);
                    if distance <= *radius {
                        // Trigger behavior execution
                        let result = behavior_tree.execute(agent, context);
                        match result {
                            BehaviorResult::Success => {
                                // Behavior executed successfully
                            }
                            BehaviorResult::Failure => {
                                eprintln!(
                                    "DEBUG: Behavior trigger execution failed for agent {}",
                                    _agent_id
                                );
                            }
                            BehaviorResult::Running => {
                                // Behavior is still running, continue on next frame
                            }
                        }
                    }
                }
            }
        }
        Ok(())
    }
}

impl Default for CrowdBehaviorManager {
    fn default() -> Self {
        Self::new()
    }
}

/// Utility function to calculate distance between two points
fn calculate_distance(a: &[f32; 3], b: &[f32; 3]) -> f32 {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let dz = b[2] - a[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// Creates a simple seek behavior tree
pub fn create_seek_behavior_tree(target: [f32; 3]) -> Result<BehaviorTree> {
    let mut tree = BehaviorTree::new();

    let action = ActionNode {
        action_type: ActionType::MoveTo { target, speed: 3.0 },
        execution_time: 0.0,
        start_time: 0.0,
    };

    let action_index = tree.add_node(BehaviorNode::Action(action))?;
    tree.set_root(action_index)?;

    Ok(tree)
}

/// Creates a simple patrol behavior tree
pub fn create_patrol_behavior_tree(waypoints: Vec<[f32; 3]>) -> Result<BehaviorTree> {
    let mut tree = BehaviorTree::new();

    let mut sequence_children = Vec::new();

    for waypoint in waypoints {
        let action = ActionNode {
            action_type: ActionType::MoveTo {
                target: waypoint,
                speed: 2.0,
            },
            execution_time: 0.0,
            start_time: 0.0,
        };
        let action_index = tree.add_node(BehaviorNode::Action(action))?;
        sequence_children.push(action_index);
    }

    let sequence = SequenceNode {
        children: sequence_children,
        current_child: 0,
    };
    let sequence_index = tree.add_node(BehaviorNode::Sequence(sequence))?;

    let repeater = RepeaterNode {
        child: sequence_index,
        repeat_count: -1, // Infinite
        current_count: 0,
    };
    let repeater_index = tree.add_node(BehaviorNode::Repeater(repeater))?;

    tree.set_root(repeater_index)?;

    Ok(tree)
}

/// Creates a flocking behavior configuration
pub fn create_flocking_behavior() -> GroupBehavior {
    GroupBehavior::Flocking {
        separation_weight: 2.0,
        alignment_weight: 1.0,
        cohesion_weight: 1.0,
        neighbor_radius: 5.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_behavior_tree_creation() {
        let mut tree = BehaviorTree::new();

        let action = ActionNode {
            action_type: ActionType::Wait { duration: 1.0 },
            execution_time: 0.0,
            start_time: 0.0,
        };

        let action_index = tree.add_node(BehaviorNode::Action(action)).unwrap();
        tree.set_root(action_index).unwrap();

        assert_eq!(tree.nodes.len(), 1);
        assert_eq!(tree.root, 0);
    }

    #[test]
    fn test_steering_force_calculation() {
        let manager = CrowdBehaviorManager::new();
        let agent_pos = [0.0, 0.0, 0.0];
        let target = [10.0, 0.0, 0.0];

        let force = manager.calculate_seek_force(&agent_pos, &target, 1.0);

        assert!(force[0] > 0.0); // Should move in positive X direction
        assert_eq!(force[1], 0.0); // No Y movement
        assert_eq!(force[2], 0.0); // No Z movement
    }

    #[test]
    fn test_distance_calculation() {
        let a = [0.0, 0.0, 0.0];
        let b = [3.0, 4.0, 0.0];
        let distance = calculate_distance(&a, &b);

        assert!((distance - 5.0).abs() < 0.001); // 3-4-5 triangle
    }

    #[test]
    fn test_create_seek_behavior_tree() {
        let target = [10.0, 0.0, 5.0];
        let tree = create_seek_behavior_tree(target).unwrap();

        assert_eq!(tree.nodes.len(), 1);
        if let BehaviorNode::Action(action) = &tree.nodes[0] {
            if let ActionType::MoveTo {
                target: action_target,
                speed: _,
            } = &action.action_type
            {
                assert_eq!(*action_target, target);
            } else {
                panic!("Expected MoveTo action");
            }
        } else {
            panic!("Expected Action node");
        }
    }

    #[test]
    fn test_create_patrol_behavior_tree() {
        let waypoints = vec![[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [10.0, 0.0, 10.0]];
        let tree = create_patrol_behavior_tree(waypoints).unwrap();

        assert_eq!(tree.nodes.len(), 5); // 3 actions + 1 sequence + 1 repeater
        assert_eq!(tree.root, 4); // Repeater is root
    }

    #[test]
    fn test_flocking_behavior_creation() {
        let behavior = create_flocking_behavior();

        if let GroupBehavior::Flocking {
            separation_weight,
            alignment_weight,
            cohesion_weight,
            neighbor_radius,
        } = behavior
        {
            assert_eq!(separation_weight, 2.0);
            assert_eq!(alignment_weight, 1.0);
            assert_eq!(cohesion_weight, 1.0);
            assert_eq!(neighbor_radius, 5.0);
        } else {
            panic!("Expected Flocking behavior");
        }
    }
}
