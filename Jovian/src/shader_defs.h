/*
 	System designed by Jeremy D. Cohen, Albert K. Lee, and Mark Bolstad, 2010-2015
 	Software designed and implemented by Mark Bolstad, 2010-2015

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

static const char *lighting_vert_source_pixel = {
	"uniform int numLights;\n"
	"uniform float attenuation;\n"
	"uniform vec4 diffuse_color;\n"
	"uniform vec4 ambient_color;\n"
	"uniform vec4 eye_pos;\n"
	"#define MAX_LIGHTS 8\n"
	"varying vec3 lightDir[MAX_LIGHTS];\n"
	"varying vec3 halfVector[MAX_LIGHTS];\n"
	"varying vec4 ambientGlobal;\n"
	"varying vec3 eye, normal;\n"
	"\n"
	"void main()\n"
	"{\n"
	"	vec4 ecPos;\n"
	"	normal = gl_NormalMatrix * gl_Normal;\n"
	"\n"
	"  /* these are the new lines of code to compute the light's direction */\n"
	"	/*ecPos = gl_ModelViewMatrix * eye_pos;\n */"
	"	ecPos = gl_ModelViewMatrix * gl_Vertex;\n "
	"  eye = ecPos.xyz;\n"
	"  int i;\n"
	"  for (i=0; i<numLights; ++i)\n"
	"  {\n"
	"	  vec3 aux = vec3(gl_LightSource[i].position-ecPos);\n"
	"	  lightDir[i] = normalize( aux );\n"
	"	  halfVector[i] = normalize( lightDir[i] + eye);\n"
	"  }\n"
	"	/* The ambient terms have been separated since one of them */\n"
	"	/* suffers attenuation */\n"
	"	ambientGlobal = ambient_color;\n"
	"	gl_Position = gl_ProjectionMatrix * gl_ModelViewMatrix * gl_Vertex;\n"
	"\n"
	"}\n"
};

static const char *lighting_frag_source_pixel = {
	"uniform int numLights;\n"
	"uniform float attenuation;\n"
	"uniform vec4 diffuse_color;\n"
	"#define MAX_LIGHTS 8\n"
	"varying vec3 lightDir[MAX_LIGHTS];\n"
	"varying vec3 halfVector[MAX_LIGHTS];\n"
	"varying vec4 ambientGlobal;\n"
	"varying vec3 eye, normal;\n"
	"\n"
	"void main()\n"
	"{\n"
	"	vec3 N,halfV,viewV,normL;\n"
	"	float NdotL,NdotHV;\n"
	"	vec4 color = ambientGlobal;\n"
	"	/* a fragment shader can't write a varying variable, hence we need\n"
	"	   a new variable to store the normalized interpolated normal */\n"
	"	N = normalize(normal);\n"
	"	/* compute the dot product between normal and normalized lightdir */\n"
	"\n"
	"  int i;\n"
	"  for (i=0; i<numLights; ++i)\n"
	"  {\n"
	"    vec3 L = normalize(lightDir[i]);\n"
	"	  NdotL = dot( N, L );\n"
	"	  if (NdotL < 0.0)\n"
	"	     NdotL = max(dot( -N, L ), 0.0 );\n"
	"\n"
	"	  color += attenuation * (gl_FrontMaterial.diffuse * diffuse_color * NdotL);\n"
	"	  halfV = normalize( halfVector[i] );\n"
	"    NdotHV = dot( N, halfV );\n"
	"	  if (NdotHV < 0.0)\n"
	"	     NdotHV = max(dot( -N, halfV ), 0.0 );\n"
	"\n"
	"	  color += attenuation * gl_FrontMaterial.specular * gl_LightSource[i].specular *\n"
	"					pow( NdotHV, gl_FrontMaterial.shininess);\n"
	"  }\n"
	"	gl_FragColor = color;\n"
	"}\n"
};

static const char *lighting_vert_source = {
	"uniform int numLights;\n"
	"uniform float attenuation;\n"
	"uniform vec4 diffuse_color;\n"
	"uniform vec4 ambient_color;\n"
	"uniform vec4 eye_pos;\n"
	"varying vec4 diffuse_value;\n"
	"void main()\n"
	"{\n"
	"  gl_Position = gl_ProjectionMatrix * gl_ModelViewMatrix * gl_Vertex;\n"
	"  diffuse_value = ambient_color;\n"
	"  vec3 N = normalize( gl_NormalMatrix * gl_Normal );\n"
	"  vec4 vVertex = gl_ModelViewMatrix * gl_Vertex;\n"
	"  int i;\n"
	"  for (i=0; i<numLights; ++i)\n"
	"  {\n"
	"    vec3 L = normalize( (gl_LightSource[i].position - vVertex).xyz );\n"
	"    diffuse_value += gl_FrontMaterial.diffuse * diffuse_color * \n"
	"     max( dot( N, L ), dot( -N, L ) ) * attenuation;\n"
	"  }\n"
	"}\n"
};

static const char *lighting_frag_source = {
	"varying vec4 diffuse_value;\n"
	"void main (void)\n"
	"{\n"
	"  gl_FragColor = diffuse_value;\n"
	"}\n"
};

