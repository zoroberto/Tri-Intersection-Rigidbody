using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace ExporterImporter
{
    public class ExporterAndImporter
    {
        // Export 
        private Vector3[] Position;
        private int[] Pair;
        private string filePath;

        public ExporterAndImporter(Vector3[] pos)
        {
            this.Position = pos;
        }

        public ExporterAndImporter(int[] p)
        {
            this.Pair = p;
        }

        /// <summary>
        ///
        /// </summary>
        public void ExportPositionsToExcel()
        {
            string objectPositions = GetObjectPositions();
            string title = "object_positions.csv";
            WriteToExcelFile(objectPositions, title);
        }

        private string GetObjectPositions()
        {
            StringBuilder sb = new StringBuilder();
            if (Position != null)
            {

                // Add headers
                //sb.AppendLine("No.,Position X,Position Y,Position Z");

                for (int i = 0; i < Position.Length; i++)
                {
                    //sb.AppendLine($"{i + 1}, {Position[i].x}, {Position[i].y}, {Position[i].z}");
                    sb.AppendLine($"{Position[i].x}, {Position[i].y}, {Position[i].z}");
                }


            }

            return sb.ToString();

        }


        // ExportCollisionAmountToExcel
        public void ExportCollisionAmountToExcel()
        {
            string objectPairs = GetObjectPairs();
            string title = "collision_amount.csv";
            WriteToExcelFile(objectPairs, title);
        }


        private string GetObjectPairs()
        {
            //Debug.Log("pair" + Pair.Length);
            StringBuilder sb = new StringBuilder();
            // Add headers
            sb.AppendLine("No.,Pair");

            for (int i = 0; i < Pair.Length; i++)
            {
                sb.AppendLine($"{i + 1}, {Pair[i]}");
            }

            return sb.ToString();
        }


        private void WriteToExcelFile(string data, string title)
        {
            filePath = Path.Combine(Application.dataPath, title);

            using (StreamWriter sw = new StreamWriter(filePath))
            {
                sw.Write(data);
                sw.Close();
            }

            Debug.Log("Data written to Excel file at: " + filePath);
        }
        
        //  Import 
        public static List<List<string>> ReadCSVFile(string fileName)
        {
            List<List<string>> csvData = new List<List<string>>();


            string filePath = Path.Combine(Application.dataPath, fileName);

            try
            {
                using (StreamReader sr = new StreamReader(filePath))
                {
                    string line;
                    while ((line = sr.ReadLine()) != null)
                    {
                        string[] rowData = line.Split(',');
                        csvData.Add(new List<string>(rowData));
                    }
                }
            }
            catch (Exception e)
            {
                Debug.LogError("Error reading CSV file: " + e.Message);
            }

            return csvData;
        }
    }
}
