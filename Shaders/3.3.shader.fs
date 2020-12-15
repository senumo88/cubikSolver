#version 330 core
out vec4 FragColor;
out vec4 FragColor1;
in vec3 ourColor;
in vec2 TexCoord;

uniform float mixValue;

// texture samplers
uniform sampler2D texture1;
uniform sampler2D texture2;


void main()
{
	// linearly interpolate between both textures (80% container, 20% awesomeface)
	FragColor = mix(texture(texture1, TexCoord), texture(texture2, TexCoord),mixValue)* vec4(ourColor, 1.0);
}