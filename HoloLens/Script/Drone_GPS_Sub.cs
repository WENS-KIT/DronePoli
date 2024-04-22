using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using RosSharp.RosBridgeClient.MessageTypes.Sensor;
using Image = UnityEngine.UI.Image;

namespace RosSharp.RosBridgeClient
{
    public class Drone_GPS_Sub : UnitySubscriber<MessageTypes.Sensor.NavSatFix>
    {
        public float messageData;
        public double drone_lon;
        public double drone_lat;
        public TextMesh CustomText1;
        public TextMesh CustomText2;
        public GameObject Drone_Object;
        private bool isMessageRecieved;
        private bool isDelay;
        public Image rangeImage;

        float drone_x, drone_y;
        protected override void Start()
        {
            base.Start();
        }

        protected override void ReceiveMessage(NavSatFix message)
        {
            drone_lon = message.longitude;
            drone_lat = message.latitude;
            isMessageRecieved = true;
        }

        private void Update()
        {
            StartCoroutine(Drone_Map_Update());
        }

        IEnumerator Drone_Map_Update()
        {
            if (isMessageRecieved == true && isDelay == false)
            {
                isDelay = true;
                CustomText2.text = drone_lon.ToString();
                CustomText1.text = drone_lat.ToString();

                Convert((double)drone_lon, (double)drone_lat);
                Drone_Object.transform.localPosition = new Vector3(drone_x, drone_y, rangeImage.transform.position.z);
            }

            yield return new WaitForSeconds(1.0f);
            isDelay = false;


        }
        private void Convert(double drone_lon, double drone_lat)
        {
            

            double user_latitude = 36.1478889476487, user_longitude = 128.394267407971;//128.38999, 36.14641;

            double map_latitude_max = 36.1489600947236;
            double map_latitude_min = user_latitude - (map_latitude_max - user_latitude);

            double map_longitude_max = 128.395593919653;
            double map_longitude_min = user_longitude - (map_longitude_max - user_longitude);


            var y = -1 + 2 * ((drone_lat - map_latitude_min) / (map_latitude_max - map_latitude_min));
            var x = -1 + 2 * ((drone_lon - map_longitude_min) / (map_longitude_max - map_longitude_min));

            drone_x = (float)x;
            drone_y = (float)y;
            
        }

        public void Up()
        {
            if (isMessageRecieved == true)
            {
                CustomText2.text = drone_lon.ToString();
                CustomText1.text = drone_lat.ToString();
            }
        }

    }
}