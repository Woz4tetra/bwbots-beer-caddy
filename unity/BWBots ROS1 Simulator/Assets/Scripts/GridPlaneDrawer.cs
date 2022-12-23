using UnityEngine;
using System.Collections;

public class GridPlaneDrawer : MonoBehaviour
{
    void Start()
    {
        int width = 100;
        int height = 100;
        Texture2D texture = new Texture2D(width, height);
        texture.wrapMode = TextureWrapMode.Repeat;
        GetComponent<Renderer>().material.mainTexture = texture;

        for (int y = 0; y < texture.height; y++)
        {
            for (int x = 0; x < texture.width; x++)
            {
                Color color;
                if (x % 100 == 0 || y % 100 == 0) {
                    color = Color.gray;
                }
                else {
                    color = Color.white;
                }
                texture.SetPixel(x, y, color);
            }
        }
        texture.Apply();

        MeshRenderer floorRenderer = GetComponent<MeshRenderer>();
        Collider floorCollider = GetComponent<Collider>();
        floorRenderer.material.mainTextureScale = new Vector2(floorCollider.bounds.size.x, floorCollider.bounds.size.z);
        floorRenderer.material.mainTextureOffset = new Vector2(0.0f, 0.0f);
    }
}
