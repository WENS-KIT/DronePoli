using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class LineDraw : MonoBehaviour
{
    private RectTransform imagerectTransform;
    private LineRenderer lr;
    public float linewidth = 1.0f;

    public GameObject S_Point;
    public GameObject E_Point;
    public List<Vector3> positionArray = new List<Vector3>();

    private Vector3 StartPos;
    private Vector3 EndPos;
    private Vector3 Pos1, Pos2;
    public List<float> x_point = new List<float>();
    public List<float> y_point = new List<float>();

    public int path_num;
    public GameObject enemy;


    private void Start()
    {
        imagerectTransform = GetComponent<RectTransform>();
        lr = GetComponent<LineRenderer>();

        //lr.startWidth = .05f;
        //lr.endWidth = .05f;
        path_num = 0;
        lr.enabled = false;
        //lr.SetPosition(0, StartPos);
       // lr.SetPosition(1, EndPos);
    }

    public float ToSingle(double value)
    {
        return (float)value;
    }

    public void lineDraw()
    {
        lr.enabled = true;
        /*
        x_point.Add(GameObject.Find("Start_Point").GetComponent<Transform>().localPosition.x);
        y_point.Add(GameObject.Find("Start_Point").GetComponent<Transform>().localPosition.y);
        Pos1 = new Vector3(GameObject.Find("Start_Point").GetComponent<Transform>().localPosition.x, GameObject.Find("Start_Point").GetComponent<Transform>().localPosition.y);
        lr.positionCount = GameObject.Find("HTML_Parsing").GetComponent<Networking>().Lon_arr.Count+5;
        //lr.SetPosition(0, StartPos);
        for (int i = 0; i < GameObject.Find("HTML_Parsing").GetComponent<Networking>().Lon_arr.Count; i ++)
        {
            x_point.Add(XConverter(GameObject.Find("HTML_Parsing").GetComponent<Networking>().Lon_data[i]));
            y_point.Add(YConverter(GameObject.Find("HTML_Parsing").GetComponent<Networking>().Lat_data[i]));
            //x_point.Add(ToSingle(GameObject.Find("HTML_Parsing").GetComponent<Networking>().Lon_arr[i] - 157694));
            //y_point.Add(ToSingle(GameObject.Find("HTML_Parsing").GetComponent<Networking>().Lat_arr[i] - 72194));
            Pos2 = new Vector3(x_point[i], y_point[i], GameObject.Find("Start_Point").GetComponent<Transform>().localPosition.z-0.05f);
            //lr.SetPosition(i, Pos1);
            lr.SetPosition(i, Pos2);
            //Pos1 = new Vector3(x_point[i-1], y_point[i-1], GameObject.Find("Start_Point").GetComponent<Transform>().localPosition.z + 0.2f);
            Debug.Log(XConverter(GameObject.Find("HTML_Parsing").GetComponent<Networking>().Lon_data[i]).ToString());
            Debug.Log(YConverter(GameObject.Find("HTML_Parsing").GetComponent<Networking>().Lat_data[i]).ToString());
            Instantiate(enemy, Pos2, Quaternion.identity);
        }
       // lr.SetPosition(GameObject.Find("HTML_Parsing").GetComponent<Networking>().Lon_arr.Count, EndPos);
        */
        path_num = GameObject.Find("Event_Manager").GetComponent<Update_Point>().positionArray.Count;
        lr.positionCount = path_num + 1;
        
        StartPos = GameObject.Find("Start_Point").GetComponent<Transform>().localPosition;
        EndPos = E_Point.transform.localPosition;
        Vector3 differenceVector = EndPos - StartPos;
        //lr.SetPosition(0, StartPos);
        //
        for (int i = 1; i < path_num; i++)
        {
            lr.SetPosition(i, GameObject.Find("Event_Manager").GetComponent<Update_Point>().positionArray[i]);
            Debug.Log("position = " + GameObject.Find("Event_Manager").GetComponent<Update_Point>().positionArray[i]);
            //Debug.Log("path num = " + path_num);
        }
        //*/
       // lr.SetPosition(2, new Vector3(-0.1f, -0.5f, -0.25f));
        //lr.SetPosition(1, new Vector3(-0.1f, -0.5f, -0.25f));
        lr.SetPosition(path_num, EndPos);
    }

    

    private void Update()
    {
        /*
        lr.positionCount = 4;
        lr.SetPosition(0, StartPos);
        lr.SetPosition(1, EndPos);
        lr.SetPosition(2, new Vector3(0.5f, 0.2f, 0));
        lr.SetPosition(3, new Vector3(0.751234f, -0.2222f, 0));
        Debug.Log("xxxxxxxxxxxxx = " + EndPos);
        */
    }

    string GetAdvertisingId()
    {
#if ENABLE_WINMD_SUPPORT
            return Windows.System.UserProfile.AdvertisingManager.AdvertisingId;
#else
        return "";
#endif
    }
}
