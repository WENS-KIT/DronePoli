using System.Collections.Generic;
using UnityEngine;
using System.IO.Ports;
using System;
using UnityEngine.UI;

public class GPSManager : MonoBehaviour
{
    
    private static GPSManager instance;
    private static GameObject Contain; 
    public SerialPort sp; 
    public static GPSManager GetInstance() 
    { 
        if (!instance) 
        { 
            Contain = new GameObject(); 
            Contain.name = "SerialPortControl"; 
            instance = Contain.AddComponent(typeof(GPSManager)) as GPSManager; 
        } return instance; 
    }
    public void OpenPort(string port, int baud, Parity parity, int bits, StopBits stopbits)
    { 
        try 
        { 
            sp = new SerialPort(port, baud, parity, bits, stopbits); 
            sp.ReadTimeout = 200; 
            sp.Handshake = Handshake.None; 
            sp.ErrorReceived += new SerialErrorReceivedEventHandler(ErrorReceive); 
            sp.DtrEnable = true; 
            sp.DataReceived += new SerialDataReceivedEventHandler(DataReceive);
            sp.Open(); Debug.Log("Silly :: OpenPort :: " + port + " open!"); } catch (Exception ex) { Debug.Log("Silly :: " + ex);
        } 
    }
    void ErrorReceive(object sender, SerialErrorReceivedEventArgs e)
    { 
        Debug.Log("Silly :: ErrorReceive :: " + e.ToString()); 
    }
    void DataReceive(object sender, SerialDataReceivedEventArgs e)
    { 
        try 
        {
            int bytes = sp.ReadByte(); Debug.Log(bytes);
        } catch (Exception ex)
        { 
            Debug.Log("Silly :: " + ex); 
        } 
    }
    public void SendData(string data)
    { 
        try
        { 
            sp.Write(data); 
        } catch (Exception ex)
        { 
            Debug.Log("Silly :: " + ex); 
        } 
    }
 
}

