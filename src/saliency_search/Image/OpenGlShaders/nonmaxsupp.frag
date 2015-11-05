// non maximal suppersion
//
// maximum of 3x3 kernel

uniform sampler2D sampler0;
uniform float testValue;

void main(void)
{
    vec4 magDir = texture2D(sampler0, gl_TexCoord[0].st);
    float alpha = 0.5/sin(3.14159/8.0); //eight directions on grid
    vec2 offset;
    offset.x = alpha * magDir.x/magDir.z;
    offset.y = alpha * magDir.y/magDir.z;

    vec4 fwdneig = texture2D(sampler0, gl_TexCoord[0].st + offset);
    vec4 backneig = texture2D(sampler0, gl_TexCoord[0].st - offset);

    if (fwdneig.z > magDir.z || backneig.z > magDir.z)
	gl_FragColor = vec4(0.0, 0.0, 0.0, 0.0);
    else
	gl_FragColor = vec4(magDir.z, 0.0, 0.0, 0.0);


     //gl_FragColor.r = magDir.z;

}


