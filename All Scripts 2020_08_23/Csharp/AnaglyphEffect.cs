using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/*
    Michael's 3-D Anaglyph effect.
    Originally written 16.10.2018

    But now this is 3D interlaced effect instead.

*/

[ExecuteInEditMode]
public class AnaglyphEffect : MonoBehaviour
{

    public Shader fxShader;
    public Camera cam2;


    private Material mat;
    private RenderTexture rt;

    // Dette är Default.
    public float _left = -0.2F;
    public float _right = 0.2F;
    public float _top = 0.2F;
    public float _bottom = -0.2F;
    public float _near = 1F;
    public float _far = 1000F;

    private void Start()
    {/*
        // Adjust camera y-angles based on stereo width.
        transform.Rotate(0,0.552f, 0);
         cam2.transform.Rotate(0, -0.552f, 0);

        transform.Translate(-0.035f, 0.0f, 0.0f);
        cam2.transform.Translate(0.035f, 0.0f, 0.0f);
       */
        // Prevent errors.
        int w = Screen.width, h = Screen.height;
        if (fxShader == null || w == 0 || h == 0)
        {
            enabled = false;
            return;
        }

        // Initialise materials used for blitting.
        mat = new Material(fxShader);
        mat.hideFlags = HideFlags.HideAndDontSave;
        cam2.enabled = false;

        // Initialse render texture.
        rt = new RenderTexture(w, h, 8, RenderTextureFormat.Default);
        cam2.targetTexture = rt;

    }

    void Update()
    { 
        if (Input.GetKey(KeyCode.I))
        {
            transform.Rotate(0.0f, 1 * Time.deltaTime, 0.0f);
            cam2.transform.Rotate(0.0f, -1 * Time.deltaTime, 0.0f);
        }
        else if (Input.GetKey(KeyCode.K))
        {
            transform.Rotate(0.0f, -1 * Time.deltaTime, 0.0f);
            cam2.transform.Rotate(0.0f, 1 * Time.deltaTime, 0.0f);
        }

        if (Input.GetKey(KeyCode.J))
        {
            transform.Translate(0.1f*Time.deltaTime, 0.0f, 0.0f);
            cam2.transform.Translate(-0.1f * Time.deltaTime, 0.0f, 0.0f);
        }
        else if (Input.GetKey(KeyCode.L))
        {
            transform.Translate(-0.1f * Time.deltaTime, 0.0f, 0.0f);
            cam2.transform.Translate(0.1f * Time.deltaTime, 0.0f, 0.0f);
        }


    }

    
    void LateUpdate()
    {
        Camera cam = Camera.main;
        Matrix4x4 m = PerspectiveOffCenter(_left, _right, _bottom, _top, _near, _far);
        Matrix4x4 m2 = PerspectiveOffCenter(-_right, -_left, _bottom, _top, _near, _far);
        cam.projectionMatrix = m;
        cam2.projectionMatrix = m2; 
    }

    static Matrix4x4 PerspectiveOffCenter(float left, float right, float bottom, float top, float near, float far)
    {
        float x = 2.0F * near / (right - left);
        float y = 2.0F * near / (top - bottom);
        float a = (right + left) / (right - left);
        float b = (top + bottom) / (top - bottom);
        float c = -(far + near) / (far - near);
        float d = -(2.0F * far * near) / (far - near);
        float e = -1.0F;
        Matrix4x4 m = new Matrix4x4();
        m[0, 0] = x;
        m[0, 1] = 0;
        m[0, 2] = a;
        m[0, 3] = 0;
        m[1, 0] = 0;
        m[1, 1] = y;
        m[1, 2] = b;
        m[1, 3] = 0;
        m[2, 0] = 0;
        m[2, 1] = 0;
        m[2, 2] = c;
        m[2, 3] = d;
        m[3, 0] = 0;
        m[3, 1] = 0;
        m[3, 2] = e;
        m[3, 3] = 0;
        return m;
    }
    

    private void OnEnable()
    {
        // Prevent errors.
        int w = Screen.width, h = Screen.height;
        if (fxShader == null || w == 0 || h == 0)
        {
            enabled = false;
            return;
        }

        // Initialise materials used for blitting.
        mat = new Material(fxShader);
        mat.hideFlags = HideFlags.HideAndDontSave;
        cam2.enabled = false;

        // Initialse render texture.
        rt = new RenderTexture(w, h, 8, RenderTextureFormat.Default);
        cam2.targetTexture = rt;
    }

    private void OnDisable()
    {
        // Clean up resources.
        if (mat != null) { DestroyImmediate(mat); }
        if (rt != null) { rt.Release(); }
        cam2.targetTexture = null;
    }

    private void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        if (cam2 == null || mat == null || rt == null)
        {
            enabled = false;
            return;
        }

        // Render to render texture
        cam2.Render();

        // Apply second texture to shader. ("_MainTex" is automatically applied by Unity3D)
        mat.SetTexture("_MainTex2", rt);

        // Blit !
        Graphics.Blit(source, destination, mat);

        // Clean up RenderTexture resources. (Not sure if this is required???)
        rt.Release();
    }
}