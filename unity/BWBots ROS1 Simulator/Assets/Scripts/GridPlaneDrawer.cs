using UnityEngine;
using System.Collections;

public class GridPlaneDrawer : MonoBehaviour
{
    void Start()
    {
        Texture2D texture = new Texture2D(1024, 1024);
        GetComponent<Renderer>().material.mainTexture = texture;

        for (int y = 0; y < texture.height; y++)
        {
            for (int x = 0; x < texture.width; x++)
            {
                Color color;
                if (x % 10 == 0 || y % 10 == 0) {
                    color = Color.gray;
                }
                else {
                    color = Color.white;
                }
                texture.SetPixel(x, y, color);
            }
        }
        texture.Apply();
    }
}