using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Net.Sockets;
using System.Net;
using System;
using System.Text;
using System.IO;
using UnityEngine.UIElements;

public class tcp_demo : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        TcpClient mySocket = new TcpClient("127.0.0.1", 6060);
        NetworkStream myStream = mySocket.GetStream();
        StreamWriter myWriter = new StreamWriter(myStream);
        Byte[] sendBytes = Encoding.UTF8.GetBytes("hello world");
        mySocket.GetStream().Write(sendBytes, 0, sendBytes.Length);
        Debug.Log(sendBytes);   
    }

    // Update is called once per frame
    void Update()
    {
    }
}
