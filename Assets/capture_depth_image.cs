using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class capture_depth_image : MonoBehaviour
{

    public string savePath = "Assets/DepthImage.png"; // Change the path and file name as needed
    private Camera attachedCamera;
    public RenderTexture depthTexture;
    public RenderTexture norm;
    public Material mat;
    private void Start()
    {
        attachedCamera = GetComponent<Camera>();
        // attachedCamera.depthTextureMode = DepthTextureMode.Depth;
    }

    private void Update()
    {
        if (true)
        {
            
            // Create a temporary texture to read the depth data into
            // Debug.Log(SystemInfo.SupportsRenderTextureFormat(RenderTextureFormat.Depth));
            // RenderTextureFormat.
            // RenderTexture depthTexture = new RenderTexture(Screen.width, Screen.height, 24, RenderTextureFormat.Depth);
            
            // RenderTexture depthTexture = new RenderTexture(Screen.width, Screen.height, 24, RenderTextureFormat.Depth);
            
            // RenderTexture target = new RenderTexture (Screen.width, Screen.height, 0, RenderTextureFormat.Default);
            // RenderTexture norm = new RenderTexture (Screen.width, Screen.height, 24, RenderTextureFormat.Default);
            // RenderTexture depthTexture = new RenderTexture (Screen.width, Screen.height, 24, RenderTextureFormat.Depth);
            // attachedCamera.SetTargetBuffers(target.colorBuffer, depthTexture.depthBuffer);
            Texture2D depthImage = new Texture2D(norm.width, norm.width, TextureFormat.RGBA32, false);
            
            RenderTexture currentRT = RenderTexture.active;

            // depthImage.read
            // Read the depth data into the texture
            // depthImage.Read
            // Convert the depth values to visible colors using the depth shader.
            Graphics.Blit(mat.mainTexture, norm, mat);
            // Debug.Log("Blitted");
            RenderTexture.active = norm;//mat.GetTexture(0);
            depthImage.ReadPixels(new Rect(0, 0, norm.width, norm.height), 0, 0);
            depthImage.Apply();
            // // Restore the previous render target and cleanup
            RenderTexture.active = currentRT;
            // attachedCamera.targetTexture = null;
            // depthTexture.Release();
            float normalizedX = 0.8f; // range [0, 1]
            float normalizedY = 1.0f; // range [0, 1]
            Color pixelColorNormalized = depthImage.GetPixelBilinear(normalizedX, normalizedY);
            Debug.Log("Color at normalized coordinates (0.5, 0.7): " + pixelColorNormalized);
            // // Encode and save the grayscale image
            byte[] bytes = depthImage.EncodeToPNG();
            File.WriteAllBytes(savePath, bytes);
            Debug.Log("Screenshot saved to: " + savePath);
        }
    }
}
