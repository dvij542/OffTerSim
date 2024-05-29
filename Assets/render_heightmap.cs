using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Drawing;
using YamlDotNet.Serialization;
using System.IO;
using UnityEngine.UIElements;
using System.Threading.Tasks;
using Unity.Barracuda;

public class render_heightmap : MonoBehaviour
{
    public Terrain terrain; // Reference to the terrain
    string imagePath; // File path to the heightmap image
    float scale;
    public float minx, miny, maxx, maxy;
    public string yamlFilePath; // File path to the YAML parameter file
    public Vector3[] centerline; // Centerline sequence of coordinates
    public Material map_;
    public GameObject[] trees;
    public float obs_avoidance_factor = 1.0f;
    public GameObject[] rocks;
    public Transform gaddi_loc;
    public Transform camera_loc;
    public float bush_demarcation;
    
    public string centerline_file;
    public float track_width;
    public float texture_blend_factor;
    public float obs_density;
    float noiseScale = 0.1f;
    public GameObject bush;
    public float vegetation_density;
    [System.Serializable]
    public class HeightmapParameters
    {
        public string filename;
        public float scale;
    }

    [System.Serializable]
    public class TrackParameters
    {
        public string centerline_file;
        public float track_width;
        public float texture_blend_factor;
        public float obs_density;
        public float vegetation_density;
        public float offtrack_roughness;
        public float scale;
        public float ox;
        public float oy;
        public float bush_demarcation;
    }

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
    
    [System.Serializable]
    public class YamlParameters
    {
        public HeightmapParameters heightmap;
        public TrackParameters track_info;
        public Location respawn_loc;
        public SensorInfo sensors;
        public float timescale;
    }

    public Vector3[] ReadCSV(string filePath, float ox, float oy, float scale)
    {
        List<Vector3> vectorList = new List<Vector3>();
        Debug.Log("Delta time: " + Time.fixedDeltaTime);
        try
        {
            // Read all lines from the CSV file
            string[] lines = File.ReadAllLines(filePath);
            minx = 2000.0f;
            miny = 2000.0f;
            maxx = -1000.0f;
            maxy = -1000.0f;
            // Parse each line and store the values in a Vector3
            foreach (string line in lines)
            {
                // Split the line by comma
                string[] values = line.Split(',');

                // Parse the values to floats
                float x, y;
                if (values.Length >= 2 && float.TryParse(values[0], out x) && float.TryParse(values[1], out y))
                {
                    minx = Mathf.Min(minx,x*scale+ox-track_width);
                    miny = Mathf.Min(miny,y*scale+oy-track_width);
                    maxx = Mathf.Max(maxx,x*scale+ox+track_width);
                    maxy = Mathf.Max(maxy,y*scale+oy+track_width);
                    // Create a new Vector3 and add it to the list
                    Vector3 vector = new Vector3(x*scale+ox, y*scale+oy, 0);
                    vectorList.Add(vector);
                }
                else
                {
                    Debug.LogWarning("Skipping line: " + line + ". It doesn't contain valid Vector3 data.");
                }
            }
        }
        catch (IOException e)
        {
            Debug.LogError("Error reading CSV file: " + e.Message);
        }

        // Convert the list to an array
        Vector3[] vectorArray = vectorList.ToArray();

        return vectorArray;
    }

    private void DeserializeParameters()
    {
        // Check if the YAML file exists
        if (!File.Exists(yamlFilePath))
        {
            Debug.LogError("YAML file not found: " + yamlFilePath);
            return;
        }

        // Deserialize YAML parameters
        string yamlContent = File.ReadAllText(yamlFilePath);
        Deserializer deserializer = new Deserializer();
        var parameters = deserializer.Deserialize<YamlParameters>(yamlContent);

        // Assign parameters
        if (parameters != null && parameters.heightmap != null && !string.IsNullOrEmpty(parameters.heightmap.filename))
        {
            imagePath = parameters.heightmap.filename;
            scale = parameters.heightmap.scale;
            centerline_file = parameters.track_info.centerline_file;
            track_width = parameters.track_info.track_width;
            texture_blend_factor = parameters.track_info.texture_blend_factor;
            obs_density = parameters.track_info.obs_density;
            vegetation_density = parameters.track_info.vegetation_density;
            noiseScale = parameters.track_info.offtrack_roughness;
            bush_demarcation = parameters.track_info.bush_demarcation;
        }
        centerline = ReadCSV(centerline_file,parameters.track_info.ox,parameters.track_info.oy,parameters.track_info.scale);
    }

    float CalculateDistance(Vector2 point1, Vector2 point2, float factor)
    {
        return Mathf.Sqrt(Mathf.Pow(point1.x * factor - point2.x, 2) + Mathf.Pow(point1.y * factor - point2.y, 2));
    }
    void Start()
    {
        if (terrain == null || string.IsNullOrEmpty(yamlFilePath))
        {
            Debug.LogError("Terrain or YAML file path not assigned!");
            return;
        }

        // Deserialize YAML parameters
        DeserializeParameters();
        Debug.Log("Deserialized parameters from YAML file: " + yamlFilePath);

        // Get terrain data
        TerrainData terrainData = terrain.terrainData;

        // Load the image from file path
        Bitmap heightmapBitmap = new Bitmap(imagePath);
        Debug.Log("Heightmap image loaded: " + imagePath);
        // Get the size of the heightmap
        int width = heightmapBitmap.Width;
        int height = heightmapBitmap.Height;

        // Get the heights array from the terrain data
        float[,] heights = new float[width, height];
        Texture2D texture = new Texture2D(width+1, height+1);
        
        // Read pixel values from the heightmap image
        float factor = 1000.0f / width;
        Debug.Log("Here 1");
        GameObject bushParent =  new GameObject("bushParent");
        float[,] is_occupied = new float[width+1, height+1];
        for (int t = 0; t<centerline.Length; t++){
            if (bush_demarcation>0.0f){
                float theta = Mathf.Atan2(centerline[(t+1)%centerline.Length].y-centerline[(t-1+centerline.Length)%centerline.Length].y,centerline[(t+1)%centerline.Length].x-centerline[(t-1+centerline.Length)%centerline.Length].x);
                float lx = centerline[t].x + track_width*Mathf.Cos(theta+Mathf.PI/2.0f);
                float ly = centerline[t].y + track_width*Mathf.Sin(theta+Mathf.PI/2.0f);
                float rx = centerline[t].x + track_width*Mathf.Cos(theta-Mathf.PI/2.0f);
                float ry = centerline[t].y + track_width*Mathf.Sin(theta-Mathf.PI/2.0f);
                if (UnityEngine.Random.value>1-bush_demarcation){
                    var dist = track_width + 1.0f;
                    for (int t_=t-10;t_<t+10;t_++){
                        float dist_ = Mathf.Pow(lx-centerline[(t_+centerline.Length)%centerline.Length].x,2)+Mathf.Pow(ly-centerline[(t_+centerline.Length)%centerline.Length].y,2);
                        dist = Mathf.Min(dist,dist_);
                    }
                    if (dist>track_width-0.01f){
                        GameObject prefabBush = Instantiate(bush, new Vector3(ly,transform.position.y,lx), Quaternion.identity, bushParent.transform); // Create a prefab tree on its pos
                        var scale = UnityEngine.Random.Range(1.5f,3.3f);   
                        prefabBush.transform.localScale = new Vector3(scale, scale, scale);
                    }
                }

                if (UnityEngine.Random.value>1-bush_demarcation){
                    var dist = track_width + 1.0f;
                    for (int t_=t-10;t_<t+10;t_++){
                        float dist_ = Mathf.Pow(rx-centerline[(t_+centerline.Length)%centerline.Length].x,2)+Mathf.Pow(ry-centerline[(t_+centerline.Length)%centerline.Length].y,2);
                        dist = Mathf.Min(dist,dist_);
                    }
                    if (dist>track_width-0.01f){
                        GameObject prefabBush = Instantiate(bush, new Vector3(ry,transform.position.y,rx), Quaternion.identity, bushParent.transform); // Create a prefab tree on its pos
                        var scale = UnityEngine.Random.Range(1.5f,3.3f);   
                        prefabBush.transform.localScale = new Vector3(scale, scale, scale);
                    }
                }

            }
            for (int i = Mathf.RoundToInt(centerline[t].x/factor-track_width/factor); i<Mathf.RoundToInt(centerline[t].x/factor+track_width/factor); i++){
                for (int j = Mathf.RoundToInt(centerline[t].y/factor-track_width/factor); j<Mathf.RoundToInt(centerline[t].y/factor+track_width/factor); j++){
                    float dist = Mathf.Sqrt(Mathf.Pow(i-centerline[t].x/factor,2) + Mathf.Pow(j-centerline[t].y/factor,2));
                    if (dist<track_width/factor)
                        is_occupied[i,j] = 1.0f;
                }
            }
        }
        Debug.Log("Here 1_");

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                // Get the grayscale value of the pixel
                
                System.Drawing.Color pixelColor = heightmapBitmap.GetPixel(x, y);
                float heightValue = pixelColor.GetBrightness(); // Use GetBrightness to convert to grayscale
                texture.SetPixel(x, height-y, new UnityEngine.Color(heightValue,0.0f,heightValue,1.0f));
                
                // Set height
                heights[x, y] = heightValue*scale;
                // if (x%4!=0 || y%4!=0){
                    
                // }
                
                // if (x*factor<minx || x*factor>maxx || y*factor<miny || y*factor>maxy){
                //     heights[x, y] += UnityEngine.Random.value * noiseScale;
                //     // Debug.Log("continue");
                //     continue;
                // }
                // float mindist = track_width + 2.0f;
                // for (int t = 0; t<centerline.Length; t++){
                //     float dist = Mathf.Sqrt(Mathf.Pow(x*factor-centerline[t].x,2) + Mathf.Pow(y*factor-centerline[t].y,2));
                //     if (dist<mindist){
                //         mindist = dist;
                //     }
                // }

                // Parallel.For(0, centerline.Length, t =>
                // {
                //     float dist = CalculateDistance(new Vector2(x, y), centerline[t], factor);
                //     // Locking is needed when accessing and updating shared variables
                //     lock (this)
                //     {
                //         if (dist < mindist)
                //         {
                //             mindist = dist;
                //         }
                //     }
                // });
                if (is_occupied[x,y]==1.0f){
                    // heights[x, y] = 0.0f;
                }
                else {
                    heights[x, y] += UnityEngine.Random.value * noiseScale;
                    
                }
                
            }
        }
        Debug.Log("Here 2");

        // Apply the heights to the terrain
        
        float[,,] maps = terrainData.GetAlphamaps(0, 0, terrain.terrainData.alphamapWidth, terrain.terrainData.alphamapHeight);
        factor = 1000.0f / terrainData.alphamapWidth;
        Debug.Log("Width, Height: " + terrainData.alphamapHeight + " " + height + " " + factor);
        for (int y = 0; y < terrainData.alphamapHeight; y++)
        {
            for (int x = 0; x < terrainData.alphamapWidth; x++)
            {
                float mindist = track_width + 2.0f;
                float a0, a1;

                if (x*factor<minx || x*factor>maxx || y*factor<miny || y*factor>maxy){
                    // heights[x, y] += UnityEngine.Random.value * noiseScale;
                    // Debug.Log("continue");
                    maps[x, y, 0] = 1.0f - texture_blend_factor;
                    maps[x, y, 1] = texture_blend_factor;
                    continue;
                }
                else{
                    Parallel.For(0, centerline.Length, t =>
                    {
                        float dist = CalculateDistance(new Vector2(x, y), centerline[t], factor);
                        // Locking is needed when accessing and updating shared variables
                        lock (this)
                        {
                            if (dist < mindist)
                            {
                                mindist = dist;
                            }
                        }
                    });
                }
                if (mindist<track_width){
                    a0 = texture_blend_factor;
                    a1 = 1.0f - texture_blend_factor;
                }
                else {
                    // heights[x, y] += UnityEngine.Random.value * noiseScale;
                    a0 = 1.0f - texture_blend_factor;
                    a1 = texture_blend_factor;
                }
                a0 += UnityEngine.Random.value * noiseScale;
                a1 += UnityEngine.Random.value * noiseScale;

                float total = a0 + a1;

                maps[x, y, 0] = a0 / total;
                maps[x, y, 1] = a1 / total;
            }
        }
        Debug.Log("Here 3");

        this.terrain.terrainData.SetAlphamaps(0,0,maps);
        TerrainData thisTerrain;
        thisTerrain = GetComponent<Terrain>().terrainData;
        texture.Apply();
        terrainData.SetHeights(0, 0, heights);

        //GameObject to be the parent
        GameObject treeParent =  new GameObject("treeParent");
 
        // For every terrain tree on the island
        factor = 1000.0f / width;
        foreach (TreeInstance terrainTree in thisTerrain.treeInstances)
        {
            // Find its local position scaled by the terrain size (to find the real world position)
            Vector3 worldTreePos = Vector3.Scale(terrainTree.position, thisTerrain.size) + Terrain.activeTerrain.transform.position;
            // terrainTree.get
            float x = worldTreePos.z;
            float y = worldTreePos.x;
            
            // float mindist = track_width + 2.0f;
            if (x<minx-50.0f || x>maxx+50.0f || y<miny-50.0f || y>maxy+50.0f){
                continue;
            }
            // for (int t = 0; t<centerline.Length; t++){
            //     float dist = Mathf.Sqrt(Mathf.Pow(x-centerline[t].x,2) + Mathf.Pow(y-centerline[t].y,2));
            //     if (dist<mindist){
            //         mindist = dist;
            //     }
                
            // }
            
            
            if (is_occupied[Mathf.RoundToInt(x/factor),Mathf.RoundToInt(y/factor)]!=1.0f){
                float r = UnityEngine.Random.Range(0.0f,1.0f);
                if (r>1.0f-vegetation_density){
                    int c = UnityEngine.Random.Range(0,trees.Length);
                    GameObject prefabTree = Instantiate(trees[c], worldTreePos, Quaternion.identity, treeParent.transform); // Create a prefab tree on its pos
                    prefabTree.transform.localScale = new Vector3(terrainTree.widthScale*4, terrainTree.heightScale*4, terrainTree.widthScale*4);
                    prefabTree.transform.rotation = Quaternion.AngleAxis(terrainTree.rotation * 57.2958f, Vector3.up);
                }
            }
            else {
                float r = UnityEngine.Random.Range(0.0f,1.0f);
                if (r>1.0f-obs_density){
                    int i = Mathf.RoundToInt(worldTreePos.z*4.096f);
                    int j = Mathf.RoundToInt(worldTreePos.x*4.096f);
                    int obs_radius = Mathf.RoundToInt(terrainTree.widthScale*obs_avoidance_factor);
                    for (int k = i-obs_radius;k<i+obs_radius+1;k++){
                        for (int l = j-obs_radius;l<j+obs_radius+1;l++){
                            float dist = Mathf.Sqrt(Mathf.Pow(k-i,2)+Mathf.Pow(l-j,2));
                            if (dist>obs_radius){
                                continue;
                            }
                            // texture.SetPixel(k, terrain_size-l, new Color(texture.GetPixel(k, terrain_size-l).r,Mathf.Min(1.0f,texture.GetPixel(k, terrain_size-l).g+Mathf.Sqrt(1.0f - dist/obs_radius)),texture.GetPixel(k, terrain_size-l).b,1.0f));
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
        Debug.Log("Here 4");

        // Apply changes to the texture
        texture.Apply();
        map_.mainTexture = texture;
        // map_.mainTexture = haha;
        // Encode the texture as a PNG and save it to the specified file path
        byte[] bytes = texture.EncodeToPNG();
        // System.IO.File.WriteAllBytes("heightmap.png", bytes);

        map_.mainTexture = texture;
        Debug.Log("Here 5");
        
        // Dispose the bitmap to release resources
        heightmapBitmap.Dispose();
    }

    // Update is called once per frame
    void Update()
    {
        camera_loc.position = new Vector3(gaddi_loc.position.x,camera_loc.position.y,gaddi_loc.position.z);
        camera_loc.rotation = Quaternion.Euler(0,gaddi_loc.rotation.eulerAngles.y,0);
    }
}
