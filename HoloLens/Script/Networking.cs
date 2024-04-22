using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;
using System.Runtime.InteropServices;
using HtmlAgilityPack;
using System.Net;
using System.Text;
using UnityEngine.UI;
using System;


public class Networking : MonoBehaviour
{
    //LineDraw
    public RectTransform imagerectTransform;
    public float linewidth = 1.0f;
    public Vector3 StartPos;
    public Vector3 EndPos;
    public Vector3 UpdatedPos = new Vector3(0, 0, -0.25f);

    //Dictionary<string, double> myDic = new Dictionary<string, double>();
    public List<double> Lon_arr = new List<double>(); //x
    public List<double> Lat_arr = new List<double>(); //y

    public static string startX = "";
    public static string startY = "";
    public static string endX = "";
    public static string endY = "";
    
    GameObject XY_Manager;

    public List<Vector3> Pos = new List<Vector3>(); 
    public static string[] split_text;
    public static double[] result_text;
    string str = "";
    public List<double> Lon_data = new List<double>();  //짝수
    public List<double> Lat_data = new List<double>();  //홀수
    public double var = 0;
    int t = 0;
    int j = 1;
    int k = 1;
    public int num = 0;

    private void Start()
    {
        imagerectTransform = GetComponent<RectTransform>();
    }
    
    public void Network()
    {
        XY_Manager = GameObject.Find("Event_Manager");
        /*
       startX = Scene_move.S_Lon;
       startY = Scene_move.S_Lat;
       endX = Scene_move.E_Lon;
       endY = Scene_move.E_Lat;
       */

        
        startX = GameObject.Find("Event_Manager").GetComponent<Update_Point>().S_lon;
        startY = GameObject.Find("Event_Manager").GetComponent<Update_Point>().S_lat;
        endX = GameObject.Find("Event_Manager").GetComponent<Update_Point>().E_lon;
        endY = GameObject.Find("Event_Manager").GetComponent<Update_Point>().E_lat;
        
        StartCoroutine(UnityWebRequestPOSTTEST());
    }


    public void XConverter(double longitude)
    {
        var k = 17.49;  //zoom's value
        var x = (long)Math.Floor((longitude + 180.0) / 360.0 * Math.Pow(2, k)) - 157694 + 3.3;
    }

    public void YConverter(double latitude)
    {
        var k = 17.49;  //zoom's value
        var y = (long)Math.Floor((1 - Math.Log(Math.Tan((Math.PI / 180) * latitude) + 1 / Math.Cos((Math.PI / 180) * latitude)) / Math.PI) / 2 * Math.Pow(2, k)) - 72194 + 1.96;
    }
    IEnumerator UnityWebRequestPOSTTEST()
    {

        Lon_data.Add(600);
        Lat_data.Add(600);
        string url = "https://apis.openapi.sk.com/tmap/routes?version=1&format=json&callback=result";
        WWWForm form = new WWWForm();
        string appKey = "l7xxc040923a75424ab29421f26ffc142c58";

        form.AddField("appKey", appKey);
        
        
        form.AddField("startX", startX);
        form.AddField("startY", startY);
        form.AddField("endX", endX);
        form.AddField("endY", endY);
        
        /*
        form.AddField("startX", "128.39411795190452");
        form.AddField("startY", "36.14790583925744");
        form.AddField("endX", "128.39379915220448");
        form.AddField("endY", " 36.14774370782758");
        */
        form.AddField("reqCoordType", "WGS84GEO");
        form.AddField("resCoordType", "WGS84GEO");
        form.AddField("searchOption", "0");
        form.AddField("trafficInfo", "Y");


        UnityWebRequest www = UnityWebRequest.Post(url, form);  // 보낼 주소와 데이터 입력

        yield return www.SendWebRequest();  // 응답 대기
        string resultData = www.downloadHandler.text;
        split_text = resultData.Split('[',  ']', ',');

        byte[] results = www.downloadHandler.data;
        Debug.Log(resultData);
        
        for(int i = 0; i < split_text.Length; i++)
        {
            if (split_text[i].Contains("."))
            {
                //double var = 0;
                
                var = double.Parse(split_text[i]);
                //double.TryParse(split_text[i], out var);
                if (t % 2 == 0)
                {
                    Lon_data.Add(var);
                   
                    Lon_arr.Add(var);
                    str = Lon_data[j].ToString();
                    j++;
                }
                else
                {
                    Lat_data.Add(var);
                  
                    Lat_arr.Add(var);
                    str = Lat_data[k].ToString();
                    k++;
                }
                t++;
                
                Debug.Log(str);    // 데이터 출력
            }
        }
        Lon_data.Add(500);
        Lat_data.Add(500);
        //Debug.Log(Lon_arr[t]);

        // Debug.Log(Lon_data.Count);
    }

    /*
    void lineDraw(Vector3 S, Vector3 E)
    {
        StartPos = S;
        EndPos = E;

        Vector3 differenceVector = E - S;

        imagerectTransform.sizeDelta = new Vector2(differenceVector.magnitude, linewidth);
        imagerectTransform.pivot = new Vector2(0, 0.5f);
        imagerectTransform.position = S;
        float angle = Mathf.Atan2(differenceVector.y, differenceVector.x) * Mathf.Rad2Deg;
        imagerectTransform.rotation = Quaternion.Euler(0, 0, angle);
    }
    */
    // Update is called once per frame
    void Update()
    {

        
    }

    }



