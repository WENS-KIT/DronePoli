using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient.MessageTypes.Std;

namespace RosSharp.RosBridgeClient
{
    public class Detected_Sub : UnitySubscriber<MessageTypes.Std.Int16>
    {
        public float messageData;
        private bool isMessageRecieved;
        private int IsDetected = 0;
        public GameObject obj;


        public string drone_lon;
        public string drone_lat;

        protected override void Start()
        {
            base.Start();
        }

        protected override void ReceiveMessage(Int16 message)
        {
            IsDetected = message.data;
            isMessageRecieved = true;
        }

        private void Update()
        {
            if (isMessageRecieved == true && IsDetected == 1)
            {
                obj.SetActive(true);
                drone_lon = GameObject.Find("ROS_Manager").GetComponent<Drone_GPS_Sub>().drone_lon.ToString();
                drone_lat = GameObject.Find("ROS_Manager").GetComponent<Drone_GPS_Sub>().drone_lat.ToString();
            }

        }
    }

}
