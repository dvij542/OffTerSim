using KartGame.KartSystems;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine;
using Random = UnityEngine.Random;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using Unity.Mathematics;
using System.IO;
using System;
using YamlDotNet.Serialization;

// using UnityEditor.ShaderGraph.Internal;


namespace KartGame.AI
{
    
    /// <summary>
    /// We only want certain behaviours when the agent runs.
    /// Training would allow certain functions such as OnAgentReset() be called and execute, while Inferencing will
    /// assume that the agent will continuously run and not reset.
    /// </summary>
    

    /// <summary>
    /// The KartAgent will drive the inputs for the KartController.
    /// </summary>
    public class SafeAgent : Agent, IInput
    {
        
#region Training Modes
        [Tooltip("Are we training the agent or is the agent production ready?")]
        public AgentMode Mode = AgentMode.Training;
        
#endregion


#region Rewards
        [Header("Rewards"), Tooltip("What penatly is given when the agent crashes?")]
        public float HitPenalty = -1f;
        public float WallHitPenalty = -1f;
        [Tooltip("Should typically be a small value, but we reward the agent for moving in the right direction.")]
        public float TowardsCheckpointReward;
        [Tooltip("Typically if the agent moves faster, we want to reward it for finishing the track quickly.")]
        public float SpeedReward;
        [Tooltip("Reward the agent when it keeps accelerating")]
        public float AccelerationReward;
        #endregion

#region ResetParams
        [Header("Inference Reset Params")]
        [Tooltip("What is the unique mask that the agent should detect when it falls out of the track?")]
        public LayerMask OutOfBoundsMask;
        [Tooltip("What are the layers we want to detect for the track and the ground?")]
        public LayerMask TrackMask;
        [Tooltip("How far should the ray be when casted? For larger karts - this value should be larger too.")]
        public float GroundCastDistance;
#endregion


        ArcadeKart m_Kart;
        bool m_Acceleration;
        bool m_Brake;
        public float added_reward;
        public int n_curr_collisions = 0;
        public float ref_angle;
        public float respawn_x = 471;
        public float respawn_y = 0;
        public float respawn_z = 0;
        public float respawn_roll = 0;
        public float respawn_pitch = 0;
        public float respawn_yaw = -140.0f;
        public float cbf_violation_reward = 10.0f;
        public float curr_height_val = 0.0f;
        public GreedyLocalController greedyLocalController;
        float m_Steering;
        public float curr_x;
        public float percent_correct;
        public float roll_factor = 1.0f;
        public float pitch_factor = 1.0f;
        string imagePath; // File path to the heightmap image
        float scale;
    
        public float smoothness_factor = 1.0f;
        public float[] neighboring_height_values = new float[11];
        // public KartGame.KartSystems.GreedyLocalController greedyLocalController;
        public bool m_EndEpisode;
        public float il_reward = 1.0f;
        public float ry;
        public int n_collisions = 0;
        public int n_wall_collisions = 0;
        public float total_collision_time = 0.0f;
        public float total_progress = 0.0f;
        public float curr_reward = 0.0f;
        public Vector3 lastVelocity;
        public float cum_unevenness = 0.0f;
        public int n_episodes = 0;
        float correct = 0.0f;
        float incorrect = 0.0f;
        public float expert_steer = 0.0f;
        float m_LastAccumulatedReward;
        public float what_penalty = 0.0f;
        public procedural_heightmap terrain_script;
        
        float prev_x = 0f;

        void Awake()
        {
            m_Kart = GetComponent<ArcadeKart>();
        }

        void Start()
        {
            // If the agent is training, then at the start of the simulation, pick a random checkpoint to train the agent.
            Debug.Log("Here 1 before");
            // OnEpisodeBegin();
            Debug.Log("Here 1");
        }

        void Update()
        {
            if (m_EndEpisode)
            {
                m_EndEpisode = false;
                // AddReward(m_LastAccumulatedReward);
                EndEpisode();
                OnEpisodeBegin();
                
            }
        }

        
        void LateUpdate()
        {
            
        }

        float Sign(float value)
        {
            if (value > 0)
            {
                return 1;
            } 
            if (value < 0)
            {
                return -1;
            }
            return 0;
        }

        public string filePath = "statistics.csv";

        // Function to append a new row of statistics to the CSV file
        public void AppendStatistics(int n_episodes, string[] statistics)
        {
            // Check if the file exists, if not, create it and add headers
            if (!File.Exists(filePath)||n_episodes==1)
            {
                string header = "Run no, No of collisions, Progress, Collision time, Cumulative unevenness, No of CBF violations, Reward"; // Add your desired headers
                File.WriteAllText(filePath, header + "\n");
            }

            // Create a string representing the new row of statistics
            string newRow = string.Join(",", statistics);

            // Append the new row to the CSV file
            File.AppendAllText(filePath, newRow + "\n");
        }
        public int n_cbf_violations = 0;

        [System.Serializable]
        public class HeightmapParameters
        {
            public string filename;
            public float scale;
        }

        public string yamlFilePath; // File path to the YAML parameter file

        [System.Serializable]
        public class Location
        {
            public float x;
            public float y;
            public float z;
            public float yaw;
            public float pitch;
            public float roll;
        }
        
        [System.Serializable]
        public class SensorInfo
        {
            public bool lidar;
            public bool camera;
            public bool pose;
            public bool imu;
        }
        
        public float angle;
        public int curr_goal;
        public GameObject[] goals;

        [System.Serializable]
        public class YamlParameters
        {
            public HeightmapParameters heightmap;
            public Location respawn_loc;
            public SensorInfo sensors;
            public float timescale;
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            // Debug.Log("Observations collected");
            // sensor.AddObservation(m_Kart.transform.position.x);
            // sensor.AddObservation(m_Kart.transform.position.y);
            // sensor.AddObservation(m_Kart.transform.position.z);
            // sensor.AddObservation(m_Kart.transform.localEulerAngles.x);
            // sensor.AddObservation(m_Kart.transform.localEulerAngles.y);
            // sensor.AddObservation(m_Kart.transform.localEulerAngles.z);
            sensor.AddObservation(m_Kart.Rigidbody.velocity.x);
            // sensor.AddObservation(m_Kart.Rigidbody.velocity.y);
            sensor.AddObservation(m_Kart.Rigidbody.velocity.z);
            // sensor.AddObservation(m_Kart.Rigidbody.angularVelocity.x);
            // sensor.AddObservation(m_Kart.Rigidbody.angularVelocity.y);
            sensor.AddObservation(m_Kart.Rigidbody.angularVelocity.z);
            // var acceleration = (m_Kart.Rigidbody.velocity - lastVelocity) / Time.fixedDeltaTime;
            // lastVelocity = m_Kart.Rigidbody.velocity;
            // sensor.AddObservation(acceleration.x);
            // sensor.AddObservation(acceleration.y);
            // sensor.AddObservation(acceleration.z);
            var relx = goals[curr_goal].transform.position.z - m_Kart.transform.position.z;
            var rely = goals[curr_goal].transform.position.x - m_Kart.transform.position.x;
            angle = Mathf.Atan2(rely,relx)-m_Kart.transform.rotation.eulerAngles.y*Mathf.PI/180.0f;
            var d = Mathf.Sqrt(Mathf.Pow(relx,2)+Mathf.Pow(rely,2));
            sensor.AddObservation(Mathf.Cos(angle));
            sensor.AddObservation(Mathf.Sin(angle));
            sensor.AddObservation(d/10.0f);
            // sensor.AddObservation(rely);

        }
        
        public float last_dist = 0.0f;
        public float goal_reach_threshold = 1.0f;
        public override void OnActionReceived(ActionBuffers actions)
        {
            // Debug.Log("Command received?");
            base.OnActionReceived(actions);
            InterpretDiscreteActions(actions);
            respawn_x = this.gameObject.transform.position.x;
            respawn_y = this.gameObject.transform.position.y+1.0f;
            respawn_z = this.gameObject.transform.position.z;
            respawn_pitch = this.gameObject.transform.rotation.eulerAngles.x;
            respawn_yaw = this.gameObject.transform.rotation.eulerAngles.y;
            respawn_roll = this.gameObject.transform.rotation.eulerAngles.z;
            var curr_dist = Mathf.Sqrt(Mathf.Pow(goals[curr_goal].transform.position.x-m_Kart.transform.position.x,2)+Mathf.Pow(goals[curr_goal].transform.position.z-m_Kart.transform.position.z,2));
            if (curr_dist < goal_reach_threshold) {
                curr_goal = (curr_goal+1)%goals.Length;
                curr_dist = Mathf.Sqrt(Mathf.Pow(goals[curr_goal].transform.position.x-m_Kart.transform.position.x,2)+Mathf.Pow(goals[curr_goal].transform.position.z-m_Kart.transform.position.z,2));
                last_dist = curr_dist;
            }

            if (Mathf.Abs(last_dist-curr_dist) < 1.0f) {
                AddReward((last_dist-curr_dist)*TowardsCheckpointReward);
                curr_reward += (last_dist-curr_dist)*TowardsCheckpointReward;
                last_dist = curr_dist;
            }
            last_dist = curr_dist;
            AddReward(m_Kart.Rigidbody.velocity.magnitude*SpeedReward);
            AddReward(m_Acceleration?AccelerationReward:0.0f);
            AddReward(HitPenalty*n_curr_collisions);    
            AddReward(WallHitPenalty*n_wall_collisions);    
            curr_reward += m_Kart.Rigidbody.velocity.magnitude*SpeedReward;
            curr_reward += m_Acceleration?AccelerationReward:0.0f;
            curr_reward += HitPenalty*n_curr_collisions;
            curr_reward += WallHitPenalty*n_wall_collisions;
        }

        
        
        public override void OnEpisodeBegin()
        {
            n_episodes += 1;
            Debug.Log("Episode: " + n_episodes);
            n_collisions = 0;
            n_wall_collisions = 0;
            n_cbf_violations = 0;
            total_collision_time = 0.0f;
            total_progress = 0.0f;
            cum_unevenness = 0.0f;
            curr_reward = 0.0f;
            correct = 0.0f;
            incorrect = 0.0f;
            switch (Mode)
            {
                case AgentMode.Training:
                    transform.rotation = Quaternion.Euler(respawn_roll,respawn_yaw,respawn_pitch);
                    transform.position = new Vector3(respawn_x,respawn_y,respawn_z);
                    m_Kart.Rigidbody.velocity = default;
                    // m_Kart.Rigidbody.velocity.x
                    m_Acceleration = false;
                    m_Brake = false;
                    m_Steering = 0f;
                    break;
                default:
                    break;
            }
        }

        void apply_cbf(){
            
        }
        
        
        

        void InterpretDiscreteActions(ActionBuffers actions)
        {
            // Debug.Log("Discrete actions received");
            m_Steering = Mathf.Clamp(actions.ContinuousActions[0],-1.0f,1.0f);//actions.DiscreteActions[0] - 1.0f;
            m_Acceleration = actions.ContinuousActions[1] > 0.0f;
            m_Brake = actions.ContinuousActions[1] < 0.0f;
        }

        public InputData GenerateInput()
        {
            return new InputData
            {
                Accelerate = m_Acceleration,
                Brake = m_Brake,
                TurnInput = m_Steering
            };
        }

        private void OnTriggerEnter(Collider other) {
            if (other.transform.tag == "Obstacle"){
                n_curr_collisions += 1;
                n_collisions += 1;
            }

        }

        private void OnCollisionEnter(Collision collision) {
            if (collision.transform.tag == "Wall"){
                n_wall_collisions += 1;
            }
            if (n_wall_collisions > 2) {
                n_wall_collisions = 2;
            }
        }
        
        private void OnCollisionExit(Collision collision) {
            if (collision.transform.tag == "Wall"){
                n_wall_collisions -= 1;
            }
            if (n_wall_collisions < 0) {
                n_wall_collisions = 0;
            }
        }

        private void OnTriggerExit(Collider other) {
            if (other.transform.tag == "Obstacle"){
                n_curr_collisions -= 1;
            }

        }
    }
}
