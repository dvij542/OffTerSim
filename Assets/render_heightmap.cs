using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Drawing;
using YamlDotNet.Serialization;
using System.IO;
using UnityEngine.UIElements;

public class render_heightmap : MonoBehaviour
{
    public Terrain terrain; // Reference to the terrain
    string imagePath; // File path to the heightmap image
    float scale;
    public string yamlFilePath; // File path to the YAML parameter file

    public Material map_;
    
    [System.Serializable]
    public class HeightmapParameters
    {
        public string filename;
        public float scale;
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
        public Location respawn_loc;
        public SensorInfo sensors;
        public float timescale;
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
        }
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
            }
        }

        // Apply the heights to the terrain
        terrainData.SetHeights(0, 0, heights);
        
        map_.mainTexture = texture;
        
        // Dispose the bitmap to release resources
        heightmapBitmap.Dispose();
    }
}
