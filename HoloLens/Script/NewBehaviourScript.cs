using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;
using UnityEditor;

public class NewBehaviourScript  : MonoBehaviour
{
    public void EndUnity()
    {
        //UnityEditor.EditorApplication.isPlaying = false;
        Application.Quit();
    }
}