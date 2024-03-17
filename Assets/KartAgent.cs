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
// using UnityEditor.ShaderGraph.Internal;


namespace KartGame.AI
{
    
    /// <summary>
    /// We only want certain behaviours when the agent runs.
    /// Training would allow certain functions such as OnAgentReset() be called and execute, while Inferencing will
    /// assume that the agent will continuously run and not reset.
    /// </summary>
    public enum AgentMode
    {
        Training,
        Inferencing
    }

    /// <summary>
    /// The KartAgent will drive the inputs for the KartController.
    /// </summary>
    public class KartAgent : Agent, IInput
    {
        
#region Training Modes
        [Tooltip("Are we training the agent or is the agent production ready?")]
        public AgentMode Mode = AgentMode.Training;
        
#endregion


#region Rewards
        [Header("Rewards"), Tooltip("What penatly is given when the agent crashes?")]
        public float HitPenalty = -1f;
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
        public float cbf_violation_reward = 10.0f;
        public float curr_height_val = 0.0f;
        public GreedyLocalController greedyLocalController;
        float m_Steering;
        public float curr_x;
        public float percent_correct;
        public float roll_factor = 1.0f;
        public float pitch_factor = 1.0f;
        public float smoothness_factor = 1.0f;
        public float[] neighboring_height_values = new float[11];
        // public KartGame.KartSystems.GreedyLocalController greedyLocalController;
        public bool m_EndEpisode;
        public float il_reward = 1.0f;
        public float ry;
        float correct = 0.0f;
        float incorrect = 0.0f;
        public float expert_steer = 0.0f;
        float m_LastAccumulatedReward;
        public float what_penalty = 0.0f;
        // public procedural_heightmap terrain_script;
        
        float prev_x = 0f;

        void Awake()
        {
            m_Kart = GetComponent<ArcadeKart>();
        }

        void Start()
        {
            // If the agent is training, then at the start of the simulation, pick a random checkpoint to train the agent.
            OnEpisodeBegin();
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

        public override void CollectObservations(VectorSensor sensor)
        {
            sensor.AddObservation(m_Kart.LocalSpeed());

            sensor.AddObservation(m_Kart.Rigidbody.velocity.normalized.x);
            sensor.AddObservation(m_Kart.Rigidbody.velocity.normalized.y);
            sensor.AddObservation(m_Kart.Rigidbody.velocity.normalized.z);
            var rx = m_Kart.transform.localEulerAngles.x;
            while (rx>180.0f) rx -= 360.0f;
            while (rx<-180.0f) rx += 360.0f;
            
            ry = m_Kart.transform.localEulerAngles.y;
            while (ry>180.0f) ry -= 360.0f;
            while (ry<-180.0f) ry += 360.0f;
            
            var rz = m_Kart.transform.localEulerAngles.z;
            while (rz>180.0f) rz -= 360.0f;
            while (rz<-180.0f) rz += 360.0f;
            
            sensor.AddObservation(rx/180.0f);
            sensor.AddObservation(m_Kart.Rigidbody.angularVelocity.y);
            sensor.AddObservation(rz/180.0f);
            // ry = m_Kart.transform.localEulerAngles.y;
            sensor.AddObservation(m_Acceleration);
            sensor.AddObservation(m_Steering);
            var x = this.gameObject.transform.position.z*4.096f;
            var y = (1000-this.gameObject.transform.position.x)*4.096f;
            var yaw = -this.gameObject.transform.rotation.eulerAngles.y*Mathf.PI/180.0f;
            // Debug.Log(m_Kart.Rigidbody.velocity.y);
            if (m_Kart.Rigidbody.velocity.y<-30.0f) m_EndEpisode = true;
            ref_angle = greedyLocalController.get_reference_angle(new Vector3(x,y,yaw));
            var rel_angle = ry*Mathf.PI/180.0f - ref_angle;
            while (rel_angle>Mathf.PI) rel_angle -= 2.0f*Mathf.PI;
            while (rel_angle<=-Mathf.PI) rel_angle += 2.0f*Mathf.PI;
            sensor.AddObservation(ref_angle);
            sensor.AddObservation(greedyLocalController.get_lateral_displacement(new Vector3(x,y,yaw)));
            var target_point = greedyLocalController.get_lookahead_point(new Vector3(x,y,yaw));
            expert_steer = -greedyLocalController.get_steer(new Vector3(x,y,yaw),target_point);
            var chs = greedyLocalController.get_lookahead_point_heights(new Vector3(x,y,yaw),0.0f);
            var nhs = greedyLocalController.get_lookahead_point_heights(new Vector3(x,y,yaw),5.0f);
            sensor.AddObservation(n_curr_collisions);
            // for(int i=0;i<11;i++){
            //     sensor.AddObservation(nhs[i]-chs[0]);
            // }
            
            // neighboring_height_values = greedyLocalController.get_lookahead_point_heights(new Vector3(x,y,yaw));
            // for(int i=0;i<11;i++){
            //     sensor.AddObservation(neighboring_height_values[i]-chs[0]);
            // }
            
            // nhs = greedyLocalController.get_lookahead_point_heights(new Vector3(x,y,yaw),15.0f);
            // for(int i=0;i<11;i++){
            //     sensor.AddObservation(nhs[i]-chs[0]);
            // }
            
            // nhs = greedyLocalController.get_lookahead_point_heights(new Vector3(x,y,yaw),20.0f);
            // for(int i=0;i<11;i++){
            //     sensor.AddObservation(nhs[i]-chs[0]);
            // }
            
            // nhs = greedyLocalController.get_lookahead_point_heights(new Vector3(x,y,yaw),25.0f);
            // for(int i=0;i<11;i++){
            //     sensor.AddObservation(nhs[i]-chs[0]);
            // }
            
            // sensor AddObservation()
        }

        public override void OnActionReceived(ActionBuffers actions)
        {
            // Debug.Log("Command received?");
            base.OnActionReceived(actions);
            InterpretDiscreteActions(actions);

            var x = this.gameObject.transform.position.z*4.096f;
            var y = (1000-this.gameObject.transform.position.x)*4.096f;
            var yaw = -this.gameObject.transform.rotation.eulerAngles.y*Mathf.PI/180.0f;
            Vector2 new_cmd = greedyLocalController.apply_cbf(new Vector3(x,y,yaw),new Vector2(m_Steering,0.0f));
            AddReward(-cbf_violation_reward*Mathf.Abs(Mathf.Min(1.0f,Mathf.Max(-1.0f,new_cmd.x))-m_Steering));
            curr_height_val = greedyLocalController.get_current_point_height(new Vector3(x,y,yaw));
            m_Steering = Mathf.Min(1.0f,Mathf.Max(-1.0f,new_cmd.x));// Find the next checkpoint when registering the current checkpoint that the agent has passed.
            
            var rx = m_Kart.transform.localEulerAngles.x;
            while (rx>180.0f) rx -= 360.0f;
            while (rx<-180.0f) rx += 360.0f;
            
            var rz = m_Kart.transform.localEulerAngles.z;
            while (rz>180.0f) rz -= 360.0f;
            while (rz<-180.0f) rz += 360.0f;
            
            AddReward(-roll_factor*Mathf.Abs(rx)/180.0f);
            AddReward(-pitch_factor*Mathf.Abs(rz)/180.0f);
            AddReward(-smoothness_factor*Mathf.Abs(m_Kart.Rigidbody.angularVelocity.y)/10.0f);
            // var next = (m_CheckpointIndex + 1) % Colliders.Length;
            // var nextCollider = Colliders[next];
            // var direction = (nextCollider.transform.position - m_Kart.transform.position).normalized;
            // var reward = Vector3.Dot(m_Kart.Rigidbody.velocity.normalized, direction);

            curr_x = m_Kart.transform.position.z;
            // var reward = curr_x - prev_x;
            var forward_vel = Mathf.Sqrt(Mathf.Pow(m_Kart.Rigidbody.velocity.x,2)+Mathf.Pow(m_Kart.Rigidbody.velocity.z,2));
            // Debug.Log("Yaw error: " + Mathf.Cos(ref_angle-this.gameObject.transform.rotation.eulerAngles.y*Mathf.PI/180.0f));
            var reward = forward_vel*Mathf.Cos(ref_angle-this.gameObject.transform.rotation.eulerAngles.y*Mathf.PI/180.0f);
            prev_x = curr_x;
            if (Mathf.Abs(reward)>10.0f){
                Debug.Log("Something wrong1!!");
                Debug.Log(reward);
                Debug.Log("Something wrong2!!");
                Debug.Log(m_Kart.LocalSpeed());
                Debug.Log("Something wrong3!!");
                Debug.Log(m_Kart.Rigidbody.velocity);
            }

            // added_reward = reward * TowardsCheckpointReward;
            // Add rewards if the agent is heading in the right direction
            AddReward(reward * TowardsCheckpointReward);
            // AddReward((m_Acceleration && !m_Brake ? 1.0f : -1.0f) * AccelerationReward);
            // AddReward(-il_reward*Mathf.Abs(expert_steer-m_Steering));
            // AddReward((m_Acceleration && !m_Brake ? 1.0f : 0.0f) * AccelerationReward);
            // AddReward(m_Kart.LocalSpeed() * SpeedReward);
            var how_much = 0.0f;
            if (m_Steering > 0.1f){
                for (int i=0;i<4;i++) how_much += neighboring_height_values[i]/4.0f;
            }
            else if(m_Steering < -0.1f){
                for (int i=7;i<11;i++) how_much += neighboring_height_values[i]/4.0f;
            } 
            else{
                for (int i=4;i<7;i++) how_much += neighboring_height_values[i]/3.0f;    
            }
            var value1 = 0.0f;
            for (int i=0;i<4;i++) value1 += neighboring_height_values[i]/4.0f;
            var value2 = 0.0f;
            for (int i=7;i<11;i++) value2 += neighboring_height_values[i]/4.0f;
            var value3 = 0.0f;
            for (int i=4;i<7;i++) value3 += neighboring_height_values[i]/3.0f;    
            
            float avg_val = (value1+value2+value3)/3.0f;
            float minval = Mathf.Min(value3,Mathf.Min(value1,value2));
            // if 
            // AddReward(Mathf.Min(value3,Mathf.Min(value1,value2))-how_much);
            added_reward = Mathf.Min(value3,Mathf.Min(value1,value2))-how_much;
            // Debug.Log(Time.fixedDeltaTime);
            // Debug.Log("wtf!");
            // AddReward(minval-how_much);
            what_penalty = Mathf.Min(value3,Mathf.Min(value1,value2))-how_much;
            if (what_penalty>-0.00001f) correct += 1;
            else incorrect += 1;
            percent_correct = correct/(correct+incorrect);
            // int ind_select = 5 - Mathf.RoundToInt(5*m_Steering);
            // AddReward(-neighboring_height_values[ind_select]);
            // AddReward(-curr_height_val);
            // AddReward(m_Kart.throttle * throotleScalingFactor); --> throttle reward can update later
        }

        
        public override void OnEpisodeBegin()
        {
            correct = 0.0f;
            incorrect = 0.0f;
            switch (Mode)
            {
                case AgentMode.Training:
                    transform.rotation = Quaternion.Euler(0.0f,0.0f,0.0f);
                    transform.position = new Vector3(respawn_x,140.6f,34.77f);
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

        void InterpretDiscreteActions(ActionBuffers actions)
        {
            // Debug.Log(actions.ContinuousActions[0]+ " " + actions.ContinuousActions[1]);
            m_Steering = actions.DiscreteActions[0] - 1.0f;
            m_Acceleration = actions.DiscreteActions[1] > -1;
            m_Brake = false;//actions.ContinuousActions[1] < 0.0f;
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

        private void OnCollisionEnter(Collision other) {
            if (other.transform.tag == "Obstacle"){
                n_curr_collisions += 1;
            }

        }

        private void OnCollisionExit(Collision other) {
            if (other.transform.tag == "Obstacle"){
                n_curr_collisions -= 1;
            }

        }
    }
}
