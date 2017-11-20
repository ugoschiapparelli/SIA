#version 410 core

uniform vec2 windowSize;
uniform sampler2D colorsTex;
uniform sampler2D normalsTex;
uniform sampler2D depthTex;
uniform vec4 light_pos;
uniform vec3 light_col;
uniform mat4 inv_projection_matrix;

out vec4 out_color;

vec3 VSPositionFromDepth(vec2 texcoord, float z)
{
    // Get x/w and y/w from the viewport position
    vec4 positionNDC = vec4(2 * vec3(texcoord, z) - 1, 1.f);
    // Transform by the inverse projection matrix
    vec4 positionVS = inv_projection_matrix * positionNDC;
    // Divide by w to get the view-space position
    return positionVS.xyz / positionVS.w;
}

vec3 shade(vec3 N, vec3 L, vec3 V,
           vec3 color, float Ka, float Kd, float Ks,
           vec3 lightCol, float shininess)
{

    vec3 final_color = color * Ka * lightCol;	//ambient

    float lambertTerm = dot(N,L);		//lambert

    if(lambertTerm > 0.0) {
        final_color += color * Kd * lambertTerm * lightCol; 	//diffuse

        vec3 R = reflect(-L,N);
        float specular = pow(max(dot(R,V), 0.0), shininess);
        final_color +=  Ks * lightCol * specular;	//specular
    }

    return final_color;
}

void main()
{
  vec2 texcoord = vec2(gl_FragCoord.x / windowSize.x, gl_FragCoord.y / windowSize.y);

  float z = texture(depthTex,texcoord).r;
  vec3 pos = VSPositionFromDepth(texcoord,z);
  vec3 l = vec3(light_pos.xyz - pos);
  vec4 vert_color = vec4(texture(colorsTex,texcoord).rgb,1.0);
  float specular_coef = texture(colorsTex,texcoord).a;
  vec3 N = texture(normalsTex,texcoord).rgb;

  out_color.rgb = shade(normalize(N), normalize(l), -normalize(pos),
                        vert_color.rgb, 0.2, 0.7, specular_coef, light_col/max(1,dot(l,l)*0.05), 5);

  out_color.a = 1.0;
  //out_color = texture(normalsTex,texcoord);
}
