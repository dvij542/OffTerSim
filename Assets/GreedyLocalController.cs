using System.Collections;
using System.Collections.Generic;
// using System.Numerics;

// using System.Numerics;
using Unity.VisualScripting;

// using System.Numerics;
using UnityEngine;

namespace KartGame.KartSystems
{

    public class GreedyLocalController : MonoBehaviour
    {
        public int radius_lookahead = 50;
        
        public float radius_obstacle = 1.0f;
        public bool cbf_applied = false;
        public float curr_pixel_val = 0.0f;
        public float curr_height_val = 0.0f;
        public float return_steer_cmd = 0.0f;
        public bool violated_right = false;
        public bool violated_left = false;
        public float yaw_diff;
        
        public float L = 2.0f*4.096f;
        public float P = 0.5f;
        public float I = 0.0001f;
        public Transform targetLocation;
        public float D = 0.1f;
        public float yaw;
        public float highest_t = 0.0f;
        float curr_cum = 0.0f;
        float target_speed = 2.0f;
        public Material heightmap_mat;
        
        // public Vector3[] obs_poses; 
        public Transform gaddi_pos;
        public procedural_heightmap terrain_script;
        public Rigidbody rig;
        float last_val = 0.0f;
        public KartGame.KartSystems.ArcadeKart veh_model;
        // Start is called before the first frame update
        void Start()
        {
            
        }

        // Update is called once per frame
        void Update()
        {   
            // gaddi_pos.rotation.eulerAngles.y
            var x = gaddi_pos.position.z*4.096f;
            var y = (1000-gaddi_pos.position.x)*4.096f;
            yaw = -gaddi_pos.rotation.eulerAngles.y*Mathf.PI/180.0f;
            var target_point = get_lookahead_point(new Vector3(x,y,yaw));
            float steer = -get_steer(new Vector3(x,y,yaw),target_point);
            float curr_val = (target_speed-rig.velocity.magnitude);
            curr_cum += curr_val;
            float throttle = Mathf.Max(0.0f,Mathf.Min(1.0f,P*curr_val + I*curr_cum + D*(curr_val - last_val)));
            last_val = curr_val;
            curr_height_val = get_current_point_height(new Vector3(x,y,yaw));
            // Pass the command
            veh_model.curr_cmd.steer = Mathf.Min(1.0f,Mathf.Max(steer,-1.0f));
            veh_model.curr_cmd.throttle = throttle;
        }

        public float get_reference_angle(Vector3 curr_pose){
            float t_ = curr_pose.x*1.0f/terrain_script.terrain_size;
            // float y_trail = (0.5f+terrain_script.a_*Mathf.Pow(t_,3) + terrain_script.b_*Mathf.Pow(t_,2) + terrain_script.c_*t_)*terrain_script.terrain_size;
            float derivative_val = -(3*terrain_script.a_*Mathf.Pow(t_,2) + terrain_script.b_*2*Mathf.Pow(t_,1) + terrain_script.c_);
            return -Mathf.Atan(derivative_val);
        }
        // public Texture2D heightmap;
        public Vector2 apply_cbf(Vector3 curr_pose, Vector2 control_cmd){
            float x = curr_pose.x;
            float y = curr_pose.y;
            Vector2 return_cmd = new Vector2(control_cmd.x,control_cmd.y);
            float t_ = x*1.0f/terrain_script.terrain_size;
            float y_trail = (0.5f-terrain_script.a_*Mathf.Pow(t_,3) - terrain_script.b_*Mathf.Pow(t_,2) - terrain_script.c_*t_)*terrain_script.terrain_size;
            float derivative_val = -(3*terrain_script.a_*Mathf.Pow(t_,2) + terrain_script.b_*2*Mathf.Pow(t_,1) + terrain_script.c_);
            float yaw = curr_pose.z;
            while (yaw>Mathf.PI) yaw-=2*Mathf.PI;
            while (yaw<-Mathf.PI) yaw+=2*Mathf.PI;
            float targt_yaw = Mathf.Atan(derivative_val);
            while (targt_yaw>Mathf.PI) targt_yaw-=2*Mathf.PI;
            while (targt_yaw<-Mathf.PI) targt_yaw+=2*Mathf.PI;
            yaw_diff = yaw - targt_yaw;
            violated_left = false;
            violated_right = false;
            if (y-y_trail > terrain_script.road_width*0.8f*terrain_script.terrain_size){
                violated_left = true;
                float target_yaw_diff = -Mathf.Min(Mathf.PI/2.0f,0.5f*((y-y_trail) - terrain_script.road_width*0.8f));
                if (yaw_diff-target_yaw_diff>0.0f){
                    return_cmd.x = Mathf.Max(control_cmd.x,(yaw_diff-target_yaw_diff)/(Mathf.PI/2.0f));
                    return_steer_cmd = (yaw_diff-target_yaw_diff)/(Mathf.PI/2.0f);
                }
            }
            else if (y-y_trail < -terrain_script.road_width*0.8f*terrain_script.terrain_size){
                violated_right = true;
                float target_yaw_diff = Mathf.Min(Mathf.PI/2.0f,0.5f*(-(y-y_trail) - terrain_script.road_width*0.8f));
                if (yaw_diff-target_yaw_diff<0.0f){
                    return_cmd.x = Mathf.Min(control_cmd.x,(yaw_diff-target_yaw_diff)/(Mathf.PI/2.0f));
                    return_steer_cmd = (yaw_diff-target_yaw_diff)/(Mathf.PI/2.0f);
                }
            }
            if (return_cmd.x!=control_cmd.x) cbf_applied = true;
            else cbf_applied = false;
            return return_cmd;
        }

        public Vector2 get_lookahead_point(Vector3 curr_pose){
            Texture2D heightmap = (Texture2D) heightmap_mat.mainTexture;
            float curr_height = heightmap.GetPixel((int)curr_pose.x, (int)curr_pose.y).r;
            curr_pixel_val = heightmap.GetPixel((int)curr_pose.x, (int)curr_pose.y).g;
            // List<Vector2> matchingPoints = new List<Vector2>();
            float min_height = float.MaxValue;
            Vector2 matchingPoint = new Vector2((int)curr_pose.x+radius_lookahead, (int)curr_pose.y+radius_lookahead);
            float yaw = curr_pose.z;
            float max_t = 0.0f;
            for (float t = -Mathf.PI/6.0f; t <= Mathf.PI/6.0f; t+=0.1f)
            {
                // Calculate the coordinates of the neighboring point
                int x = (int)(curr_pose.x + radius_lookahead*Mathf.Cos(yaw+t));
                int y = (int)(curr_pose.y + radius_lookahead*Mathf.Sin(yaw+t));

                if (x >= 0 && x < heightmap.width && y >= 0 && y < heightmap.height)
                {   
                    // Condition to find point on the trail only
                    bool isTrail = (heightmap.GetPixel(x, y).b != 0.0f);
                    if (!isTrail) {
                        continue;
                    }
                    

                    // Condition to find point in the forward semi-circle of car only
                    float t_ = x*1.0f/terrain_script.terrain_size;
                    float y_trail = (0.5f-terrain_script.a_*Mathf.Pow(t_,3) - terrain_script.b_*Mathf.Pow(t_,2) - terrain_script.c_*t_)*terrain_script.terrain_size;

                    float derivative_val = -(3*terrain_script.a_*Mathf.Pow(t_,2) + terrain_script.b_*2*Mathf.Pow(t_,1) + terrain_script.c_);
                    float extra_cost = 0.0f;
                    if ((Mathf.Cos(yaw+t) + Mathf.Sin(yaw+t)*derivative_val)/Mathf.Sqrt(1+Mathf.Pow(derivative_val,2)) < 0.2f){
                        // continue;
                        extra_cost += 100.0f*Mathf.Abs((Mathf.Cos(yaw+t) + Mathf.Sin(yaw+t)*derivative_val)/Mathf.Sqrt(1+Mathf.Pow(derivative_val,2))-0.2f);
                    }

                    
                    extra_cost += 10000.0f*heightmap.GetPixel(x, y).g;
                    float neighborHeight = heightmap.GetPixel(x, y).r + extra_cost;
                    float height = neighborHeight;//Mathf.Abs(neighborHeight - curr_height);
                    // Check if the height values match
                    if(height < min_height)
                    {
                        max_t = t;
                        // Add the matching point to the list
                        min_height = height;
                        matchingPoint = new Vector2(x,y);
                    }
                }
                
            }
            float x_ = matchingPoint.x;
            float y_ = matchingPoint.y;
            targetLocation.position = new Vector3(1000.0f - y_ * 1000.0f/4096.0f,targetLocation.position.y,x_*1000.0f/4096.0f);
            highest_t = max_t;
            return matchingPoint;
        }
        
        public float get_lateral_displacement(Vector3 curr_pose){
            float t_ = curr_pose.x*1.0f/terrain_script.terrain_size;
            
            float y_trail = (0.5f-terrain_script.a_*Mathf.Pow(t_,3) - terrain_script.b_*Mathf.Pow(t_,2) - terrain_script.c_*t_)*terrain_script.terrain_size;
            return (curr_pose.y - y_trail);
        }

        public float[] get_lookahead_point_heights(Vector3 curr_pose,double rl){
            Texture2D heightmap = (Texture2D) heightmap_mat.mainTexture;
            float curr_height = heightmap.GetPixel((int)curr_pose.x, (int)curr_pose.y).r;
            curr_pixel_val = heightmap.GetPixel((int)curr_pose.x, (int)curr_pose.y).g;
            // List<Vector2> matchingPoints = new List<Vector2>();
            float yaw = curr_pose.z;
            var heightvals = new float[11];
            int i = 0;
            for (float t = -Mathf.PI/6.0f; t <= Mathf.PI/6.0f; t+=Mathf.PI/30.0f)
            {
                // Calculate the coordinates of the neighboring point
                int x = (int)(curr_pose.x + rl*Mathf.Cos(yaw+t));
                int y = (int)(curr_pose.y + rl*Mathf.Sin(yaw+t));

                if (x >= 0 && x < heightmap.width && y >= 0 && y < heightmap.height)
                {   
                    // Condition to find point on the trail only
                    bool isTrail = (heightmap.GetPixel(x, y).b != 0.0f);
                    if (!isTrail) {
                        heightvals[i++] = 5.0f;
                        continue;
                    }
                    

                    // Condition to find point in the forward semi-circle of car only
                    float t_ = x*1.0f/terrain_script.terrain_size;
                    float y_trail = (0.5f+terrain_script.a_*Mathf.Pow(t_,3) + terrain_script.b_*Mathf.Pow(t_,2) + terrain_script.c_*t_)*terrain_script.terrain_size;

                    float derivative_val = -(3*terrain_script.a_*Mathf.Pow(t_,2) + terrain_script.b_*2*Mathf.Pow(t_,1) + terrain_script.c_);
                    float extra_cost = 0.0f;
                    if ((Mathf.Cos(yaw+t) + Mathf.Sin(yaw+t)*derivative_val)/Mathf.Sqrt(1+Mathf.Pow(derivative_val,2)) < 0.2f){
                        // continue;
                        extra_cost += Mathf.Abs((Mathf.Cos(yaw+t) + Mathf.Sin(yaw+t)*derivative_val)/Mathf.Sqrt(1+Mathf.Pow(derivative_val,2))-0.2f);
                    }

                    
                    extra_cost += 3.0f*heightmap.GetPixel(x, y).g;
                    heightvals[i++] = heightmap.GetPixel(x, y).r + extra_cost;
                    
                }
                
            }
            
            return heightvals;
        }

        public float[] get_lookahead_point_heights(Vector3 curr_pose){
            Texture2D heightmap = (Texture2D) heightmap_mat.mainTexture;
            float curr_height = heightmap.GetPixel((int)curr_pose.x, (int)curr_pose.y).r;
            curr_pixel_val = heightmap.GetPixel((int)curr_pose.x, (int)curr_pose.y).g;
            // List<Vector2> matchingPoints = new List<Vector2>();
            float yaw = curr_pose.z;
            var heightvals = new float[11];
            int i = 0;
            for (float t = -Mathf.PI/6.0f; t <= Mathf.PI/6.0f; t+=Mathf.PI/30.0f)
            {
                // Calculate the coordinates of the neighboring point
                int x = (int)(curr_pose.x + radius_lookahead*Mathf.Cos(yaw+t));
                int y = (int)(curr_pose.y + radius_lookahead*Mathf.Sin(yaw+t));

                if (x >= 0 && x < heightmap.width && y >= 0 && y < heightmap.height)
                {   
                    // Condition to find point on the trail only
                    bool isTrail = (heightmap.GetPixel(x, y).b != 0.0f);
                    if (!isTrail) {
                        heightvals[i++] = 5.0f;
                        continue;
                    }
                    

                    // Condition to find point in the forward semi-circle of car only
                    float t_ = x*1.0f/terrain_script.terrain_size;
                    float y_trail = (0.5f+terrain_script.a_*Mathf.Pow(t_,3) + terrain_script.b_*Mathf.Pow(t_,2) + terrain_script.c_*t_)*terrain_script.terrain_size;

                    float derivative_val = -(3*terrain_script.a_*Mathf.Pow(t_,2) + terrain_script.b_*2*Mathf.Pow(t_,1) + terrain_script.c_);
                    float extra_cost = 0.0f;
                    if ((Mathf.Cos(yaw+t) + Mathf.Sin(yaw+t)*derivative_val)/Mathf.Sqrt(1+Mathf.Pow(derivative_val,2)) < 0.2f){
                        // continue;
                        extra_cost += Mathf.Abs((Mathf.Cos(yaw+t) + Mathf.Sin(yaw+t)*derivative_val)/Mathf.Sqrt(1+Mathf.Pow(derivative_val,2))-0.2f);
                    }

                    
                    extra_cost += 3.0f*heightmap.GetPixel(x, y).g;
                    heightvals[i++] = heightmap.GetPixel(x, y).r + extra_cost;
                    
                }
                
            }
            
            return heightvals;
        }
        
        public float get_current_point_height(Vector3 curr_pose){
            Texture2D heightmap = (Texture2D) heightmap_mat.mainTexture;
            float curr_height = heightmap.GetPixel((int)curr_pose.x, (int)curr_pose.y).r;
            curr_pixel_val = heightmap.GetPixel((int)curr_pose.x, (int)curr_pose.y).g;
            // List<Vector2> matchingPoints = new List<Vector2>();
            float yaw = curr_pose.z;
            var heightvals = 10.0f;
            {
                // Calculate the coordinates of the neighboring point
                int x = (int)(curr_pose.x+ 3.0*Mathf.Cos(yaw));
                int y = (int)(curr_pose.y+ 3.0*Mathf.Sin(yaw));

                if (x >= 0 && x < heightmap.width && y >= 0 && y < heightmap.height)
                {   
                    // Condition to find point on the trail only
                    bool isTrail = (heightmap.GetPixel(x, y).b != 0.0f);
                    if (!isTrail) {
                        return 3.0f;
                    }
                    

                    // Condition to find point in the forward semi-circle of car only
                    float t_ = x*1.0f/terrain_script.terrain_size;
                    float y_trail = (0.5f+terrain_script.a_*Mathf.Pow(t_,3) + terrain_script.b_*Mathf.Pow(t_,2) + terrain_script.c_*t_)*terrain_script.terrain_size;

                    float derivative_val = -(3*terrain_script.a_*Mathf.Pow(t_,2) + terrain_script.b_*2*Mathf.Pow(t_,1) + terrain_script.c_);
                    float extra_cost = 0.0f;
                    // if ((Mathf.Cos(yaw) + Mathf.Sin(yaw)*derivative_val)/Mathf.Sqrt(1+Mathf.Pow(derivative_val,2)) < 0.2f){
                    //     // continue;
                    //     extra_cost += Mathf.Abs((Mathf.Cos(yaw) + Mathf.Sin(yaw)*derivative_val)/Mathf.Sqrt(1+Mathf.Pow(derivative_val,2))-0.2f);
                    // }

                    
                    extra_cost += 2.0f*heightmap.GetPixel(x, y).g;
                    heightvals = heightmap.GetPixel(x, y).r + extra_cost;
                    
                }
                
            }
            
            return heightvals;
        }
        
        public float get_steer(Vector3 curr_pos, Vector2 target_point){
            float dx_ = target_point.x - curr_pos.x;
            float dy_ = target_point.y - curr_pos.y;
            float yaw = curr_pos.z;
            float dx = dx_*Mathf.Cos(yaw) + dy_*Mathf.Sin(yaw);
            float dy = -dx_*Mathf.Sin(yaw) + dy_*Mathf.Cos(yaw);
            float steer = 2*L*dy/((Mathf.Pow(dx,2)+Mathf.Pow(dy,2)));
            return steer;
        }
    }
}