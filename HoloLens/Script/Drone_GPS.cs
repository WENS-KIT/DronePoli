using System.Collections;
using UnityEngine;
using UnityEngine.Networking;
using UnityEngine.UI;
using RosSharp.RosBridgeClient;

public class Drone_GPS : MonoBehaviour
{
    public Image GPS_Image;
    UnityEngine.UI.Image _img;
    double result;
    double user_Lon = 1;
    double user_Lat = 1;
    string[] S_markers = { "128.39423951", "36.14244921" };
    double markers;
    double move = 0.0000001;
    string url;
    void Start()
    {

        string url1 = "https://apis.openapi.sk.com/tmap/staticMap?version=1&appkey=l7xxc040923a75424ab29421f26ffc142c58&coordType=WGS84GEO&width=512&height=512&zoom=17&format=PNG&longitude=128.44224&latitude=36.14176";
        //longitude=128.39423935431742&latitude=36.14787971597255
        Download(url1);
    }
    private void Update()
    {
        /*
        user_Lon = Path_Sub.user_lon;
        user_Lat = Path_Sub.user_lat;
        string url1 = "https://apis.openapi.sk.com/tmap/staticMap?version=1&appkey=l7xxc040923a75424ab29421f26ffc142c58&coordType=GRS80GEO&width=512&height=512&zoom=15&format=PNG&longitude="; // 128.38776,36.14510,   128.39196&latitude=36.1468800496401&markers="; // 128.393096923828,36.1467467778144";
        string url2 = user_Lon.ToString(); 
        string url3 = "&latitude=";
        string url4 = user_Lat.ToString(); 
        string url = url1 + url2 + url3 + url4 + "&markers=" + GameObject.Find("Event_Manager").GetComponent<Update_Point>().E_lon + "," + GameObject.Find("Event_Manager").GetComponent<Update_Point>().E_lat;
        //Download(url);
        StartCoroutine(UnityWebRequestGetTest(url));
        */
    }

    public void User_Update(double U_lon, double U_lat)
    {
        string url1 = "https://apis.openapi.sk.com/tmap/staticMap?version=1&appkey=l7xxc040923a75424ab29421f26ffc142c58&coordType=GRS80GEO&width=512&height=512&zoom=15&format=PNG&longitude="; // 128.39196&latitude=36.1468800496401&markers="; // 128.393096923828,36.1467467778144";
        string url2 = U_lon.ToString();
        string url3 = "&latitude=";
        string url4 = U_lat.ToString();
        string url5 = "&markers=" + GameObject.Find("ROS_Manager").GetComponent<Detected_Sub>().drone_lon + "," + GameObject.Find("ROS_Manager").GetComponent<Detected_Sub>().drone_lat;
        string url = url1 + url2 + url3 + url4 + "&markers=" + GameObject.Find("Event_Manager").GetComponent<Update_Point>().E_lon + "," + GameObject.Find("Event_Manager").GetComponent<Update_Point>().E_lat;
        //Download(url);
        //StartCoroutine(UnityWebRequestGetTest(url));
        UnityWebRequestGetTest(url);
    }

    public void Download(string url)
    {
        StartCoroutine(UnityWebRequestGetTest(url));
    }

    IEnumerator UnityWebRequestGetTest(string url)
    {
        UnityWebRequest www = UnityWebRequestTexture.GetTexture(url);
        DownloadHandlerTexture texDl = new DownloadHandlerTexture(true);
        www.downloadHandler = texDl;
        yield return www.SendWebRequest();  // 응답 대기


            if (!(www.isNetworkError || www.isHttpError))
            {
                Texture2D t = texDl.texture;
                Sprite s = Sprite.Create(t, new Rect(0, 0, t.width, t.height), Vector2.zero, 1f);
                GPS_Image.sprite = s;
             }
                
            
            else
            {
                Debug.Log(www.error);
            }



    }

}
