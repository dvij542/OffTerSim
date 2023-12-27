using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
// 5. () Point to Learning on the left tab and select Karting Microgame (This is to install all libraries related to it including ML-agents as I have not included Library folder on github due to very large size). Make sure version 2021.3 is selected in case you have installed multiple versions.

public class save_path : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
    }

    public Transform objectToRecord; // The object whose position you want to record
    public int recordFrequency = 1000; // Number of updates before saving to CSV
    public string filePath = "traj_followed.csv"; // File path to save the CSV
    public char delimiter = ','; // CSV delimiter character

    private List<Vector3> positionData = new List<Vector3>();
    public int updateCount = 0;

    // Update is called once per frame
    private void Update()
    {
        // Record the object's position in each FixedUpdate
        positionData.Add(objectToRecord.position);

        updateCount++;

        if (updateCount >= recordFrequency)
        {
            SavePositionDataToCSV();
            updateCount = 0;
        }
    }

    private void SavePositionDataToCSV()
    {
        int dataCount = positionData.Count;
        if (dataCount == 0)
        {
            Debug.LogWarning("No position data to save.");
            return;
        }

        string csv = "X" + delimiter + "Y" + delimiter + "Z\n";

        foreach (Vector3 position in positionData)
        {
            csv += position.x + "," + position.y + "," + position.z + "\n";
        }

        File.WriteAllText(filePath, csv);

        Debug.Log("Position data saved to: " + filePath);
        // positionData.Clear();
    }
    
}
