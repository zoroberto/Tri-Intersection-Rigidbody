//using Assets.Scripts;
//using System.Collections;
//using System.Collections.Generic;
//using UnityEngine;

//public class ExportCollidableAmount : MonoBehaviour
//{
//    private List<int> collidableAmount = new List<int>();
//    public int MaxFrame = 100;

//    // export collidable amount
//    private void ExportCollidableAmt()
//    {

//        if (collidableAmount.Count > MaxFrame)
//        {
//            ExporterAndImporter exporter = new ExporterAndImporter(collidableAmount.ToArray());
//            exporter.ExportCollisionAmountToExcel();
//            UnityEditor.EditorApplication.isPlaying = false;
//        }

//    }

//    private void Start()
//    {



//    }

//    // Update is called once per frame
//    void Update()
//    {

//        //collidableAmount.Add(GetComponent<Controller>().collidablePairIndex.Count);
//        //print(" Amount  " + collidableAmount.Count / MaxFrame);
//        ExportCollidableAmt();
//    }
//}