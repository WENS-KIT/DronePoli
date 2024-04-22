using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class Move_Drone : MonoBehaviour
{
    public GameObject Drone;
    public Image Map;
    int isStart = 0;
    Vector3 dest_1 = new Vector3(0.2f, 0, 0);
    Vector3 dest_2 = new Vector3(0.217f, 0.4f, 0);
    Vector3 dest_3 = new Vector3(0.4f, 0.4f, 0);

    private void Start()
    {
        isStart = 0;
    }

    private void Update()
    {
        if(isStart == 1)
        {
            Drone.transform.localPosition = Vector3.MoveTowards(Drone.transform.localPosition, dest_1, Time.deltaTime / (5f * 17f)); //(5.5f * 16f) 싱크가 너무 딱 맞음
            if(Drone.transform.localPosition.x == 0.2f)
            {
                isStart = 2;
            }
        }
        else if(isStart == 2)
        {
            Drone.transform.localRotation = Quaternion.Euler(0, 0, 0);
            isStart = 3;
        }

        else if(isStart == 3)
        {
            Drone.transform.localPosition = Vector3.MoveTowards(Drone.transform.localPosition, dest_2, Time.deltaTime / (5.7f * 22f));
            if (Drone.transform.localPosition.y == 0.4f)
            {
                isStart = 4;
            }
        }

        else if(isStart == 4)
        {
            Drone.transform.localRotation = Quaternion.Euler(0, 0, -90);
            isStart = 5;
        }
        else if(isStart == 5)
        {
            Drone.transform.localPosition = Vector3.MoveTowards(Drone.transform.localPosition, dest_3, Time.deltaTime / (5.5f * 16f));
        }
        
    }

    public void Drone_Move()
    {
        StartCoroutine(Wait());
    }

    IEnumerator iRotation(float targetAngle)
    {
        float st = Drone.transform.eulerAngles.z;
        float er = targetAngle;

        float t = 0f;

        while (t < 1f)
        {
            t += Time.deltaTime;
            float speed = t;
            float zRotation = Mathf.Lerp(st, er, speed) % 360.0f;

            Drone.transform.eulerAngles = new Vector3(Drone.transform.eulerAngles.x, Drone.transform.eulerAngles.y, zRotation);
            // Debug.Log(yRotation + " " + transform.eulerAngles.y);

            yield return null;
        }
    }

    IEnumerator Run(float duration, Vector3 dest, Vector3 Start)
    {
        var runTime = 0.0f;
        float speed = 7.8f;
        runTime += Time.deltaTime;
        Drone.transform.localPosition = Vector3.MoveTowards(Start, dest,  runTime / (speed * duration));
        yield return null;
    }

    IEnumerator Wait()
    {
        yield return new WaitForSeconds(11f);
        isStart = 1;
    }

}
