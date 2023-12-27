Shader "Custom/DepthImageShader1"
{
    Properties
    {
        _MainTex ("Texture", 2D) = "white" {}
        
    }
    SubShader
    {
        // No culling or depth
        Cull Off ZWrite Off ZTest Always

        Pass
        {
            CGPROGRAM
            #pragma vertex vert // compile function vert as vertex shader
            #pragma fragment frag // compile function frag as fragment shader

            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct v2f
            {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
            };

            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = v.uv;
                return o;
            }

            sampler2D _MainTex;
            sampler2D _CameraDepthTexture;

            fixed4 frag (v2f i) : SV_Target
            {
                float depth = tex2D(_CameraDepthTexture, i.uv).g;
                //depth = Linear01Depth(depth);
                //depth = depth * _ProjectionParams.z;
                fixed4 col = tex2D(_CameraDepthTexture, i.uv);
                col.r = 0.0;//depth;
                col.g = 1.0;//depth;
                col.b = 0.0;//depth;
                
                return col;
            }
            ENDCG
        }
    }
}