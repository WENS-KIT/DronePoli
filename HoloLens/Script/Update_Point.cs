using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class Update_Point : MonoBehaviour
{
    public GameObject rangeObject;
    public Image rangeImage;
    public GameObject S_Point;
    public GameObject E_Point;
    public GameObject Updated_Point;
    string a, b;
    public List<Vector3> positionArray = new List<Vector3>();
    public string S_lon, S_lat, E_lon, E_lat;
    
    MeshCollider meshCollider;
    Vector3 worldPosition;

    // Start is called before the first frame update
    void Start()
    {

        S_Point.transform.position = new Vector3(
       Mathf.Clamp(S_Point.transform.position.x, rangeImage.transform.position.x, rangeImage.transform.position.x),
       Mathf.Clamp(S_Point.transform.position.y, rangeImage.transform.position.y, rangeImage.transform.position.y),
        rangeImage.transform.position.z
      );
        E_Point.transform.position = new Vector3(
        Mathf.Clamp(E_Point.transform.position.x, -rangeObject.transform.position.x - 1f, +rangeObject.transform.position.x + 1f),
        Mathf.Clamp(E_Point.transform.position.y, -rangeObject.transform.position.y - 1f, +rangeObject.transform.position.y + 1f),
        rangeObject.transform.position.z - 0.1f
       );
    }

    // Update is called once per frame
    void Update()
    {

        // rectTransform.position = Input.mousePosition;
        //Vector3 mouseInput1 = Camera.main.ScreenToWorldPoint(Input.mousePosition + new Vector3(0, 0, 4f));

        S_Point.transform.position = new Vector3(
       Mathf.Clamp(S_Point.transform.position.x, rangeImage.transform.position.x, rangeImage.transform.position.x),
       Mathf.Clamp(S_Point.transform.position.y, rangeImage.transform.position.y, rangeImage.transform.position.y),
        rangeImage.transform.position.z
      );
        E_Point.transform.position = new Vector3(
        Mathf.Clamp(E_Point.transform.position.x, -rangeObject.transform.position.x, +rangeObject.transform.position.x + 5f),
        Mathf.Clamp(E_Point.transform.position.y, -rangeObject.transform.position.y, +rangeObject.transform.position.y + 5f),
        rangeObject.transform.position.z - 0.1f
       );


        Debug.Log(E_Point.transform.localPosition);
        a = S_Point.transform.localPosition.ToString();
        b = E_Point.transform.localPosition.ToString();

    }

    public void Path_Active()
    {
        var S_x = S_Point.transform.localPosition.x + 232484.65;
        //18.0.5 >  316485
        var S_y = -S_Point.transform.localPosition.y + 106432.95;
        //18.05 > 144888
        var z = 18.05;  //zoom's value

        var S_longitude = S_x / Math.Pow(2, z) * 360 - 180;
        var S_n = Math.PI - 2 * Math.PI * S_y / Math.Pow(2, z);
        var S_latitude = 180 / Math.PI * Math.Atan(0.5 * (Math.Exp(S_n) - Math.Exp(-S_n)));
        //double S_longitude = 128.393520581942;
        //double S_latitude = 36.147484050222;

        var E_x = (E_Point.transform.localPosition.x) + 232484.65;
        var E_y = (-E_Point.transform.localPosition.y) + 106432.95;

        //var E_x = 232486.15;
        //var E_y = 106431.45;

        var E_longitude = E_x / Math.Pow(2, z) * 360 - 180;
        var E_n = Math.PI - 2 * Math.PI * E_y / Math.Pow(2, z);
        var E_latitude = 180 / Math.PI * Math.Atan(0.5 * (Math.Exp(E_n) - Math.Exp(-E_n)));

        var x = (int)Math.Floor((128.39423935431742 + 180.0) / 360.0 * Math.Pow(2, z));
        var y = (int)Math.Floor((1 - Math.Log(Math.Tan((Math.PI / 180) * 36.14787971597255) + 1 / Math.Cos((Math.PI / 180) * 36.14787971597255)) / Math.PI) / 2 * Math.Pow(2, z));

        /*
        var E_x1 = 232483.6; //-
        var E_y1 = 106431.85; //-


        var E_longitude1 = E_x1 / Math.Pow(2, z) * 360 - 180;
        var E_n1 = Math.PI - 2 * Math.PI * E_y1 / Math.Pow(2, z);
        var E_latitude1 = 180 / Math.PI * Math.Atan(0.5 * (Math.Exp(E_n1) - Math.Exp(-E_n1)));

        var E_x2 = 232485.6; // +
        var E_y2 = 106431.85; //-


        var E_longitude2 = E_x2 / Math.Pow(2, z) * 360 - 180;
        var E_n2 = Math.PI - 2 * Math.PI * E_y2 / Math.Pow(2, z);
        var E_latitude2 = 180 / Math.PI * Math.Atan(0.5 * (Math.Exp(E_n2) - Math.Exp(-E_n2)));

        var E_x3 = 232483.6; //-
        var E_y3 = 106433.85; // +


        var E_longitude3 = E_x3 / Math.Pow(2, z) * 360 - 180;
        var E_n3 = Math.PI - 2 * Math.PI * E_y3 / Math.Pow(2, z);
        var E_latitude3 = 180 / Math.PI * Math.Atan(0.5 * (Math.Exp(E_n3) - Math.Exp(-E_n3)));
        var E_x4 = 232485.6; //+
        var E_y4 = 106433.85; // +


        var E_longitude4 = E_x4 / Math.Pow(2, z) * 360 - 180;
        var E_n4 = Math.PI - (2 * Math.PI * E_y4 / Math.Pow(2, z));
        var E_latitude4 = 180 / Math.PI * Math.Atan(0.5 * (Math.Exp(E_n4) - Math.Exp(-E_n4)));

        var E_x5 = 232484.6;
        var E_y5 = 106432.85;


        var E_longitude5 = E_x5 / Math.Pow(2, z) * 360 - 180;
        var E_n5 = Math.PI - 2 * Math.PI * E_y5 / Math.Pow(2, z);
        var E_latitude5 = 180 / Math.PI * Math.Atan(0.5 * (Math.Exp(E_n5) - Math.Exp(-E_n5)));



       // var x = (int)Math.Floor((128.39423935431742 + 180.0) / 360.0 * Math.Pow(2, z));
       // var y = (int)Math.Floor((1 - Math.Log(Math.Tan((Math.PI / 180) * 36.14787971597255) + 1 / Math.Cos((Math.PI / 180) * 36.14787971597255)) / Math.PI) / 2 * Math.Pow(2, z));

        

        Debug.Log("longitude1 = " + E_longitude1);
        Debug.Log("latitude1 = " + E_latitude1);
        Debug.Log("longitude2 = " + E_longitude2);
        Debug.Log("latitude2 = " + E_latitude2);
        Debug.Log("longitude3 = " + E_longitude3);
        Debug.Log("latitude3 = " + E_latitude3);
        Debug.Log("longitude4 = " + E_longitude4);
        Debug.Log("latitude4 = " + E_latitude4);
        Debug.Log("longitude5 = " + E_longitude5);
        Debug.Log("latitude5 = " + E_latitude5);
        Debug.Log("x = " + x);
        Debug.Log("y = " + y);
        */

        S_lon = S_longitude.ToString();
        S_lat = S_latitude.ToString();
        E_lon = E_longitude.ToString();
        E_lat = E_latitude.ToString();

        Debug.Log("longitude = " + S_longitude);
        Debug.Log("latitude = " + S_latitude);
        Debug.Log("x = " + S_x);
        Debug.Log("y = " + S_y);
    }
    public void Path_Active2()
    {
        var S_x = S_Point.transform.localPosition.x + 232484.65;
        //18.0.5 >  316485
        var S_y = -S_Point.transform.localPosition.y + 106432.95;
        //18.05 > 144888
        var z = 18.05;  //zoom's value

        var S_longitude = S_x / Math.Pow(2, z) * 360 - 180;
        var S_n = Math.PI - 2 * Math.PI * S_y / Math.Pow(2, z);
        var S_latitude = 180 / Math.PI * Math.Atan(0.5 * (Math.Exp(S_n) - Math.Exp(-S_n)));
        //double S_longitude = 128.393520581942;
        //double S_latitude = 36.147484050222;

        var E_x = (E_Point.transform.localPosition.x) + 232484.65;
        var E_y = (-E_Point.transform.localPosition.y) + 106432.95;

        //var E_x = 232486.15;
        //var E_y = 106431.45;

        var E_longitude = E_x / Math.Pow(2, z) * 360 - 180;
        var E_n = Math.PI - 2 * Math.PI * E_y / Math.Pow(2, z);
        var E_latitude = 180 / Math.PI * Math.Atan(0.5 * (Math.Exp(E_n) - Math.Exp(-E_n)));


        for (int i = 1; i < GameObject.Find("HTML_Parsing").GetComponent<Networking>().Lon_data.Count - 1; i++)
        {
            float map_x, map_y;

            double user_latitude = 36.1474604847227, user_longitude = 128.393537826546;//128.38999, 36.14641;
                                                                                      //latitude = 36.1469725481039
                                                                                      //longitude = 128.39500153424

            //double map_latitude_max = 36.1489600947236;
            double map_latitude_max = 36.1485316376491;
            double map_latitude_min = user_latitude - (map_latitude_max - user_latitude) ;
            


            //double map_longitude_max = 128.395593919653;
            double map_longitude_max = 128.394864338228;
            double map_longitude_min = user_longitude - (map_longitude_max - user_longitude);

            double gps_latitude = GameObject.Find("HTML_Parsing").GetComponent<Networking>().Lat_data[i];
            double gps_longitude = GameObject.Find("HTML_Parsing").GetComponent<Networking>().Lon_data[i];
            
            var y =  -1 + 2 * ((gps_latitude - map_latitude_min) / (map_latitude_max - map_latitude_min));
            var x = -1 + 2 * ((gps_longitude - map_longitude_min) / (map_longitude_max - map_longitude_min));
            /*
            var y = -1 + 2 * ((gps_latitude - map_latitude_min) / (map_latitude_max - map_latitude_min));
            var x = 1 + 0.5 * ((gps_longitude - map_longitude_min) / (map_longitude_max - map_longitude_min));
             
            var x = (int)Math.Floor((gps_longitude + 180.0) / 360.0 * Math.Pow(2, z)) - 317583.83;
            var y = (int)Math.Floor((1 - Math.Log(Math.Tan((Math.PI / 180) * gps_latitude) + 1 / Math.Cos((Math.PI / 180) * gps_latitude)) / Math.PI) / 2 * Math.Pow(2, z)) - 145391.18;
            */

            positionArray.Add(new Vector3((float)x, (float)y, 0));
            Debug.Log("num = " + positionArray.Count);
            Debug.Log("x da = " + (float)x);
            Debug.Log("y dd= " + (float)y);

        }
    }
        
            int radius = 6371; // Earth Radius in Km

}
