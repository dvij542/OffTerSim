using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;

// using UnityEditor.ShaderGraph.Internal;
using UnityEngine;

public class procedural_heightmap : MonoBehaviour
{
    // Start is called before the first frame update
    public Terrain terrain;
    public float max_h = 1.0f;
    public float freq_x = 0.1f;
    public float freq_y = 0.2f;
    public float phase = 0.5f;
    public int terrain_size = 1024;
    public int n_bumps = 100;
    public float safe_dist = 1.0f;
    public float a = 1.0f;
    public float b = -1.0f;
    public float c = 0.4f;
    public float obs_avoidance_factor = 1.0f;
    public float a_;
    public float b_;
    public float c_;
    public float right_slope = 1.0f;
    public float left_slope = 1.0f;
    public float road_width = 0.03f;
    public GameObject[] trees;
    public Material map_;
    public Texture haha;
    public GameObject[] rocks;
    public Transform gaddi_loc;
    public Transform camera_loc;
    void AddBump(float height, int spread, int x, int y, float[,] mesh){
        for (int i = x-spread; i < x+spread; i++){
            for (int j = y-spread; j < y+spread; j++){
                float dist = Mathf.Pow(i-x,2) + Mathf.Pow(j-y,2);
                mesh[i,j] += height*Mathf.Exp(-dist/spread);
            }    
        }
    }
    public void Start()
    {   
        var mesh = new float[terrain_size+1,terrain_size+1];
        var maxval = 0.0f;
        a_ = UnityEngine.Random.Range(-a*1.0f,a*1.0f);
        b_ = UnityEngine.Random.Range(-b*1.0f,b*1.0f);
        c_ = UnityEngine.Random.Range(-c*1.0f,c*1.0f);
        var road_widths = new float[terrain_size+1];
        for (int k = 0; k < n_bumps + 1; k++)
        {
            phase = UnityEngine.Random.Range(0.0f,2*Mathf.PI);
            freq_x = UnityEngine.Random.Range(0.0f,0.2f);
            freq_y = UnityEngine.Random.Range(0.0f,0.2f);
            float max_h_sample = UnityEngine.Random.Range(0.0f,max_h);
            for (int i = 0; i < terrain_size + 1; i++){
                if (k==0) {
                    road_width += UnityEngine.Random.Range(-road_width/400.0f,road_width/400.0f);
                    road_widths[i] = road_width;
                }
                float t = i*1.0f/terrain_size;
                float y = (0.5f+a_*Mathf.Pow(t,3) + b_*Mathf.Pow(t,2) + c_*(t))*terrain_size;
                // road_width += UnityEngine.Random.Range(-road_width/400.0f,road_width/400.0f);
                // float diff = (j - y)/terrain_size;
                road_width = road_widths[i];
                // for (int j = Mathf.Max(0,Mathf.RoundToInt(y-road_width*terrain_size)); j < Mathf.Min(terrain_size + 1,Mathf.RoundToInt(y+road_width*terrain_size)); j++){
                for (int j = 0; j < terrain_size + 1; j++){
                    mesh[i,j] += (max_h_sample+max_h_sample*Mathf.Sin(phase+i*freq_x+j*freq_y))/100.0f;
                    maxval = Mathf.Max(maxval,mesh[i,j]);
                }        
            }
        }
        // var arr = new float[4096,2];
        // for (int i = 0; i<4096; i+=1){
        //     float x = (i/4096.0f)*terrain_size;
        //     float t = i/4096.0f;
        //     float y = (a*Mathf.Pow(t,2) + b*t + c)*terrain_size;
        // }
        Texture2D texture = new Texture2D(terrain_size+1, terrain_size+1);
        for (int i = 0; i < terrain_size + 1; i++){
            float t = i*1.0f/terrain_size;
            float y = (0.5f+a_*Mathf.Pow(t,3) + b_*Mathf.Pow(t,2) + c_*(t))*terrain_size;
            // road_width += UnityEngine.Random.Range(-road_width/400.0f,road_width/400.0f);
            road_width = road_widths[i];
            for (int j = 0; j < terrain_size + 1; j++){
                float diff = (j - y)/terrain_size;
                texture.SetPixel(i, terrain_size-j, new Color(1.0f,0.0f,0.0f,1.0f));
                
                if (diff>road_width){
                    int j_ = Mathf.Max(0,Mathf.Min(terrain_size + 1,Mathf.RoundToInt(y+road_width*terrain_size))-1);
                    mesh[i,j] = mesh[i,j_];
                    mesh[i,j] += (diff-road_width)*right_slope+ UnityEngine.Random.Range(-0.0003f,0.0003f);
                }
                else if (diff<-road_width){
                    int j_ = Mathf.Min(terrain_size,Mathf.Max(0,Mathf.RoundToInt(y-road_width*terrain_size)));
                    mesh[i,j] = mesh[i,j_];
                    mesh[i,j] += 0.2f - (diff+road_width)*left_slope + UnityEngine.Random.Range(-0.0003f,0.0003f);
                }
                else {
                    texture.SetPixel(i, terrain_size-j, new Color(mesh[i,j]/maxval,0.0f,mesh[i,j]/maxval,1.0f));
                    if (diff<-(road_width-safe_dist)|| diff > (road_width-safe_dist)){
                        float viol = Mathf.Max(diff-(road_width-safe_dist),-diff-(road_width-safe_dist))/safe_dist;
                        texture.SetPixel(i, terrain_size-j, new Color(mesh[i,j]/maxval,viol,mesh[i,j]/maxval,1.0f));
                    }

                    mesh[i,j] += 0.2f;
                }
                if (mesh[i,j] > 0.8f) mesh[i,j] = 0.8f;
                if (mesh[i,j] < 0.0f) mesh[i,j] = 0.0f;
            }
        }
        
        // Destroy the temporary texture to free up memory
        // Destroy(texture);
        // for (int i=0;i<n_bumps;i++){
        //     AddBump(UnityEngine.Random.Range(0.0f,max_h/200.0f),UnityEngine.Random.Range(35,65),UnityEngine.Random.Range(65,terrain_size-65),UnityEngine.Random.Range(65,terrain_size-65),mesh);
        // } 
        float[,,] maps = terrain.terrainData.GetAlphamaps(0, 0, terrain.terrainData.alphamapWidth, terrain.terrainData.alphamapHeight);
        float noiseScale = 0.1f;
        for (int y = 0; y < terrain.terrainData.alphamapHeight; y++)
        {
            for (int x = 0; x < terrain.terrainData.alphamapWidth; x++)
            {
                float t = x*1.0f/terrain.terrainData.alphamapWidth;
                float y_ = (0.5f+a_*Mathf.Pow(t,3) + b_*Mathf.Pow(t,2) + c_*(t))*terrain.terrainData.alphamapWidth;
                float diff = (y - y_)/terrain.terrainData.alphamapWidth;
                float a0, a1;
                if (diff>road_width){
                    a0 = 2.0f*(diff-road_width);//maps[x, y, 0];
                    if (a0>1.0f) a0 = 1.0f; 
                    a1 = 1.0f-a0;//maps[x, y, 1];
                }
                else if (diff<-road_width){
                    a0 = -2.0f*(diff+road_width);//maps[x, y, 0];
                    if (a0<0.0f) a0 = 0.0f; 
                    a1 = 1.0f-a0;//maps[x, y, 1];
                }
                else {
                    a0 = 0.9f;
                    a1 = 0.1f;
                }
                
                a0 += UnityEngine.Random.value * noiseScale;
                a1 += UnityEngine.Random.value * noiseScale;

                float total = a0 + a1;

                maps[x, y, 0] = a0 / total;
                maps[x, y, 1] = a1 / total;
            }
        }

        this.terrain.terrainData.SetAlphamaps(0, 0, maps);
        this.terrain.terrainData.SetHeights(0,0,mesh);   
        TerrainData thisTerrain;
        thisTerrain = GetComponent<Terrain>().terrainData;
        texture.Apply();

        //GameObject to be the parent
        GameObject treeParent =  new GameObject("treeParent");
 
        // For every terrain tree on the island
        foreach (TreeInstance terrainTree in thisTerrain.treeInstances)
        {
            // Find its local position scaled by the terrain size (to find the real world position)
            Vector3 worldTreePos = Vector3.Scale(terrainTree.position, thisTerrain.size) + Terrain.activeTerrain.transform.position;
            // terrainTree.get
            float t = worldTreePos.z/1000.0f;
            float y = worldTreePos.x;
            float y_ = (0.5f+a_*Mathf.Pow(t,3) + b_*Mathf.Pow(t,2) + c_*(t))*1000;
            float diff = (y - y_)/1000.0f;
            if (diff>road_width||diff<-road_width){
                float r = UnityEngine.Random.Range(0.0f,1.0f);
                if (r>0.8f){
                    int c = UnityEngine.Random.Range(0,trees.Length);
                    GameObject prefabTree = Instantiate(trees[c], worldTreePos, Quaternion.identity, treeParent.transform); // Create a prefab tree on its pos
                    prefabTree.transform.localScale = new Vector3(terrainTree.widthScale*4, terrainTree.heightScale*4, terrainTree.widthScale*4);
                    prefabTree.transform.rotation = Quaternion.AngleAxis(terrainTree.rotation * 57.2958f, Vector3.up);
                }
            }
            else {
                float r = UnityEngine.Random.Range(0.0f,1.0f);
                if (r>0.5f){
                    int i = Mathf.RoundToInt(worldTreePos.z*4.096f);
                    int j = Mathf.RoundToInt(worldTreePos.x*4.096f);
                    int obs_radius = Mathf.RoundToInt(terrainTree.widthScale*obs_avoidance_factor);
                    for (int k = i-obs_radius;k<i+obs_radius+1;k++){
                        for (int l = j-obs_radius;l<j+obs_radius+1;l++){
                            float dist = Mathf.Sqrt(Mathf.Pow(k-i,2)+Mathf.Pow(l-j,2));
                            if (dist>obs_radius){
                                continue;
                            }
                            texture.SetPixel(k, terrain_size-l, new Color(texture.GetPixel(k, terrain_size-l).r,Mathf.Min(1.0f,texture.GetPixel(k, terrain_size-l).g+Mathf.Sqrt(1.0f - dist/obs_radius)),texture.GetPixel(k, terrain_size-l).b,1.0f));
                        }
                    }
                    // for (int k=i-)
                    int c = UnityEngine.Random.Range(0,rocks.Length);
                    GameObject prefabTree = Instantiate(rocks[c], worldTreePos, Quaternion.identity, treeParent.transform); // Create a prefab tree on its pos
                    prefabTree.transform.localScale = new Vector3(terrainTree.widthScale, terrainTree.heightScale, terrainTree.widthScale);
                    prefabTree.transform.rotation = Quaternion.AngleAxis(terrainTree.rotation * 57.2958f, Vector3.up);
                }
                else {
                    // terrainTree.
                }
            }
            // Create the new tree out of the prefab
 
            //Then set the new tree to the UnityEngine.Randomized size and rotation of the terrain tree
        }
        // Apply changes to the texture
        texture.Apply();
        map_.mainTexture = texture;
        // map_.mainTexture = haha;
        // Encode the texture as a PNG and save it to the specified file path
        byte[] bytes = texture.EncodeToPNG();
        System.IO.File.WriteAllBytes("heightmap.png", bytes);

        // Then delete all the terrain trees on the island
        // this.terrain.treeDistance = 0;
        // thisTerrain.SetTreeInstances(newTrees.ToArray(),false);
    }

    // Update is called once per frame
    void Update()
    {
        camera_loc.position = new Vector3(gaddi_loc.position.x,camera_loc.position.y,gaddi_loc.position.z);
        camera_loc.rotation = Quaternion.Euler(0,gaddi_loc.rotation.eulerAngles.y,0);
    }
}
