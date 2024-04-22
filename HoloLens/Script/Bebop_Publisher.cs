using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class Bebop_Publisher : UnityPublisher<MessageTypes.Std.Int16>
    {
        private short takeoff = 1;
        private short land = 2;
        private short move = 3;
        private short plan = 4;
        private short stop = 9;

        // double 형 배열 double route[lon, lat, lon, lat, ...]
        private MessageTypes.Std.Int16 message;
       

        protected override void Start()
        {
            base.Start();
            //InitializeMessage();
        }

        private void FixedUpdate()
        {
          //  UpdateMessage();
        }
        /*
        private void InitializeMessage()
        {
            // route 1
            message = new MessageTypes.Std.Int32
            {
                data = takeoff
            };
        }
        */
        public void InitializeTakeoffMessage()
        {
            // route 1
            message = new MessageTypes.Std.Int16
            {
                data = takeoff
            };
            message.data = takeoff;

            Publish(message);
        }
        public void InitializeLandMessage()
        {
            // route 1
            message = new MessageTypes.Std.Int16
            {
                data = land
            };

            message.data = land;

            Publish(message);
        }

        public void InitializeMoveMessage()
        {
            // route 1
            message = new MessageTypes.Std.Int16
            {
                data = move
            };
            message.data = move;

            Publish(message);
        }

        public void InitializePlanMessage()
        {
            // route 1
            message = new MessageTypes.Std.Int16
            {
                data = plan
            };
            message.data = plan;

            Publish(message);
        }
        public void InitializeStopMessage()
        {
            // route 1
            message = new MessageTypes.Std.Int16
            {
                data = stop
            };
            message.data = stop;

            Publish(message);
        }
        /*
        private void UpdateMessage()
        {
            message.data = order;

            Publish(message);
        }
        */

    }
}