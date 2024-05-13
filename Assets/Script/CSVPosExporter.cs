using ExporterImporter;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CSVPosExporter : MonoBehaviour
{
    [Header("Number model")]
    public int number_object = 1;
    //public GameObject clone_object;

    [Header("Random Range")]
    public Vector3 rangeMin = new Vector3(-10f, 0f, 0f);
    public Vector3 rangeMax = new Vector3(10f, 10f, 20f);

    private List<Vector3> object_position = new List<Vector3>();

    void Start()
    {
        GenerateObjectPosition();
        ExportPosition();
    }

    private void GenerateObjectPosition()
    {
        HashSet<Vector3> generatedPositions = new HashSet<Vector3>();

        for (int i = 0; i < number_object; i++)
        {
            Vector3 randomPosition;

            do
            {
                // Generate random position within the specified range
                float x = Random.Range(rangeMin.x, rangeMax.x);
                float y = Random.Range(rangeMin.y, rangeMax.y);
                float z = Random.Range(rangeMin.z, rangeMax.z);

                randomPosition = new Vector3(x, y, z);

                //Instantiate(clone_object, randomPosition, transform.rotation);
                object_position.Add(randomPosition);

            } while (generatedPositions.Contains(randomPosition));
        }
    }

    private void ExportPosition()
    {
        ExporterAndImporter exporter = new ExporterAndImporter(object_position.ToArray());
        exporter.ExportPositionsToExcel();
    }
}
