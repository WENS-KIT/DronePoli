using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class Path_Pub : UnityPublisher<MessageTypes.Sensor.NavSatFix>
    {
        private MessageTypes.Sensor.NavSatFix message;
        protected override void Start()
        {
            base.Start();
        }
        
        public void UpdateMessage()
        {
            message = new MessageTypes.Sensor.NavSatFix();
            message.header.Update();
            for(int i = 0;i <  GameObject.Find("HTML_Parsing").GetComponent<Networking>().Lon_data.Count; i++)
           {
         
                message.latitude = GameObject.Find("HTML_Parsing").GetComponent<Networking>().Lat_data[i];
                message.longitude = GameObject.Find("HTML_Parsing").GetComponent<Networking>().Lon_data[i];
                Publish(message);
             }
        }
    }
}

