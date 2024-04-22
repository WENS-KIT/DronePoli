using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;
using System.Runtime.InteropServices;
using HtmlAgilityPack;
using System.Net;
using System.Text;
using UnityEngine.UI;
using RosSharp.RosBridgeClient.MessageTypes.Sensor;
using Image = UnityEngine.UI.Image;

namespace RosSharp.RosBridgeClient
{
    public class Path_Sub : UnitySubscriber<MessageTypes.Sensor.NavSatFix>
    {
        public float messageData;
        double user_lon;
        double user_lat;
        public TextMesh CustomText1;
        public TextMesh CustomText2;
        private bool isMessageRecieved;
        public Image GPS_Image;
        UnityEngine.UI.Image _img;
        double result;
        double user_Lon = 1;
        double user_Lat = 1;
        string[] S_markers = { "128.39423951", "36.14244921" };
        double markers;
        double move = 0.0000001;
        protected override void Start()
        {
            base.Start();
            
        }

        protected override void ReceiveMessage(NavSatFix message)
        {
            user_lon = message.longitude;
            user_lat = message.latitude;
            isMessageRecieved = true;
        }

        private void Update()
        {
            /*
            if (isMessageRecieved == true)
            {
                User_Update(user_lon, user_lat);
                CustomText2.text = "Good";
            }*/
        }

        public void Up()
        {
            if (isMessageRecieved == true)
            {
                User_Update(user_lon, user_lat);
                CustomText2.text = user_lon.ToString();
            }
            else
                Debug.LogError("Error");
        }

        public void User_Update(double U_lon, double U_lat)
        {
            //string url1 = "https://apis.openapi.sk.com/tmap/staticMap?version=1&appkey=l7xxc040923a75424ab29421f26ffc142c58&coordType=GRS80GEO&width=512&height=512&zoom=15&format=PNG&longitude=128.38999&latitude=36.14641&markers=128.38905,36.14609";
            
            string url1 = "https://apis.openapi.sk.com/tmap/staticMap?version=1&appkey=l7xxc040923a75424ab29421f26ffc142c58&coordType=GRS80GEO&width=512&height=512&zoom=15&format=PNG&longitude="; // 128.39196&latitude=36.1468800496401&markers="; // 128.393096923828,36.1467467778144";
            string url2 = U_lon.ToString("F5");
            string url3 = "&latitude=";
            string url4 = U_lat.ToString("F5");
            string url = url1 + url2 + url3 + url4 + "&markers=128.39206,36.14621";//GameObject.Find("Event_Manager").GetComponent<Update_Point>().E_lon + "," + GameObject.Find("Event_Manager").GetComponent<Update_Point>().E_lat;
            
            //string url = url1;
            Download(url);
            //StartCoroutine(UnityWebRequestGetTest(url));
            //UnityWebRequestGetTest(url);
            /*
            string url1 = "https://apis.openapi.sk.com/tmap/staticMap?version=1&appkey=l7xxc040923a75424ab29421f26ffc142c58&coordType=GRS80GEO&width=512&height=512&zoom=15&format=PNG&longitude=128.39196&latitude=36.1468800496401&markers=128.393096923828,36.1467467778144";
            Download(url1);
            */
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
}

/*
  if (user_lon != null)
            {
                Debug.Log("Path_Sub Error!!!");
                CustomText2.text = "Path_Sub Error!!!";
            }
            else
            {
                GameObject.Find("HTML_Parsing").GetComponent<Drone_GPS>().User_Update(user_lon, user_lat);
                CustomText2.text = "Good";
            }
*/

