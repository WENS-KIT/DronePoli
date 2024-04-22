using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;
using UnityEngine.UI;
using System.Text;

public class Search_Point : MonoBehaviour
{
    public InputField Start_input;
    public InputField End_input;
    string Start_Point;
    string End_Point;
    public string S_Lon;
    public string S_Lat;
    public string E_Lon;
    public string E_Lat;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    public void PointSearch()
    {
        string url1 = "https://apis.openapi.sk.com/tmap/geo/convertAddress?version=1&reqMulti=S&resCoordType=WGS84GEO&searchTypCd=NtoO&callback=result&appKey=l7xxc040923a75424ab29421f26ffc142c58&reqAdd="; 
        
        Start_Point = UnityWebRequest.EscapeURL(Start_input.text);
        End_Point = UnityWebRequest.EscapeURL(End_input.text);


        string url3 = url1 + Start_Point; 
        string url2 = url1 + End_Point;
        StartCoroutine(UnityWebRequestGet(url3, 3)); //Start
        StartCoroutine(UnityWebRequestGet(url2, 2)); //End
    }

    IEnumerator UnityWebRequestGet(string url, int n)
    {
        UnityWebRequest www = UnityWebRequest.Get(url); // GET 형식

        yield return www.SendWebRequest();  // 응답 대기

        string resultData = www.downloadHandler.text;
        resultData = End_input.text;
        string test1 = www.downloadHandler.text.Substring(www.downloadHandler.text.IndexOf("newLat")+9, "newLat".Length + 5);
        string test2 = www.downloadHandler.text.Substring(www.downloadHandler.text.IndexOf("newLon")+9, "newLon".Length + 6);
        if (n == 3)
        {
            S_Lon = test1;
            S_Lat = test2;
        }
        else
        {
            E_Lon = test1;
            E_Lat = test2;
        }
        Debug.Log(test1);
        Debug.Log(test2);
    }

        // Update is called once per frame
        void Update()
    {
        
    }
}
