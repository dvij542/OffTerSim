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
            // Texture2D depthImage = new Texture2D(Screen.width, Screen.height, TextureFormat.RFloat, false);
            
            // RenderTexture currentRT = RenderTexture.active;

            // depthImage.read
            // Read the depth data into the texture
            // depthImage.Read
            // Convert the depth values to visible colors using the depth shader.
            Graphics.Blit(mat.mainTexture, norm, mat);
            // Debug.Log("Blitted");
            // RenderTexture.active = norm;//mat.GetTexture(0);
            // depthImage.ReadPixels(new Rect(0, 0, norm.width, norm.height), 0, 0);
            // // Restore the previous render target and cleanup
            // RenderTexture.active = currentRT;
            // attachedCamera.targetTexture = null;
            // // depthTexture.Release();

            // // Encode and save the grayscale image
            // byte[] bytes = depthImage.EncodeToPNG();
            // File.WriteAllBytes(savePath, bytes);
        }
    }
}
