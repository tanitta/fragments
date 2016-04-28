xof 0302txt 0064
template Header {
 <3D82AB43-62DA-11cf-AB39-0020AF71E433>
 WORD major;
 WORD minor;
 DWORD flags;
}

template Vector {
 <3D82AB5E-62DA-11cf-AB39-0020AF71E433>
 FLOAT x;
 FLOAT y;
 FLOAT z;
}

template Coords2d {
 <F6F23F44-7686-11cf-8F52-0040333594A3>
 FLOAT u;
 FLOAT v;
}

template Matrix4x4 {
 <F6F23F45-7686-11cf-8F52-0040333594A3>
 array FLOAT matrix[16];
}

template ColorRGBA {
 <35FF44E0-6C7C-11cf-8F52-0040333594A3>
 FLOAT red;
 FLOAT green;
 FLOAT blue;
 FLOAT alpha;
}

template ColorRGB {
 <D3E16E81-7835-11cf-8F52-0040333594A3>
 FLOAT red;
 FLOAT green;
 FLOAT blue;
}

template IndexedColor {
 <1630B820-7842-11cf-8F52-0040333594A3>
 DWORD index;
 ColorRGBA indexColor;
}

template Boolean {
 <4885AE61-78E8-11cf-8F52-0040333594A3>
 WORD truefalse;
}

template Boolean2d {
 <4885AE63-78E8-11cf-8F52-0040333594A3>
 Boolean u;
 Boolean v;
}

template MaterialWrap {
 <4885AE60-78E8-11cf-8F52-0040333594A3>
 Boolean u;
 Boolean v;
}

template TextureFilename {
 <A42790E1-7810-11cf-8F52-0040333594A3>
 STRING filename;
}

template Material {
 <3D82AB4D-62DA-11cf-AB39-0020AF71E433>
 ColorRGBA faceColor;
 FLOAT power;
 ColorRGB specularColor;
 ColorRGB emissiveColor;
 [...]
}

template MeshFace {
 <3D82AB5F-62DA-11cf-AB39-0020AF71E433>
 DWORD nFaceVertexIndices;
 array DWORD faceVertexIndices[nFaceVertexIndices];
}

template MeshFaceWraps {
 <4885AE62-78E8-11cf-8F52-0040333594A3>
 DWORD nFaceWrapValues;
 Boolean2d faceWrapValues;
}

template MeshTextureCoords {
 <F6F23F40-7686-11cf-8F52-0040333594A3>
 DWORD nTextureCoords;
 array Coords2d textureCoords[nTextureCoords];
}

template MeshMaterialList {
 <F6F23F42-7686-11cf-8F52-0040333594A3>
 DWORD nMaterials;
 DWORD nFaceIndexes;
 array DWORD faceIndexes[nFaceIndexes];
 [Material]
}

template MeshNormals {
 <F6F23F43-7686-11cf-8F52-0040333594A3>
 DWORD nNormals;
 array Vector normals[nNormals];
 DWORD nFaceNormals;
 array MeshFace faceNormals[nFaceNormals];
}

template MeshVertexColors {
 <1630B821-7842-11cf-8F52-0040333594A3>
 DWORD nVertexColors;
 array IndexedColor vertexColors[nVertexColors];
}

template Mesh {
 <3D82AB44-62DA-11cf-AB39-0020AF71E433>
 DWORD nVertices;
 array Vector vertices[nVertices];
 DWORD nFaces;
 array MeshFace faces[nFaces];
 [...]
}

Header{
1;
0;
1;
}

Mesh {
 379;
 150.00000;2.50000;0.00000;,
 297.11780;2.50000;29.26353;,
 300.00000;2.50000;-0.00001;,
 288.58194;2.50000;57.40249;,
 274.72046;2.50000;83.33553;,
 256.06601;2.50000;106.06602;,
 233.33551;2.50000;124.72045;,
 207.40250;2.50000;138.58194;,
 179.26352;2.50000;147.11780;,
 149.99994;2.50000;150.00000;,
 300.00000;2.50000;-0.00001;,
 297.11780;2.50000;29.26353;,
 346.15704;27.50000;39.01804;,
 350.00000;27.50000;-0.00002;,
 288.58194;2.50000;57.40249;,
 334.77594;27.50000;76.53666;,
 274.72046;2.50000;83.33553;,
 316.29395;27.50000;111.11404;,
 256.06601;2.50000;106.06602;,
 291.42136;27.50000;141.42136;,
 233.33551;2.50000;124.72045;,
 261.11401;27.50000;166.29393;,
 207.40250;2.50000;138.58194;,
 226.53665;27.50000;184.77592;,
 179.26352;2.50000;147.11780;,
 189.01801;27.50000;196.15706;,
 149.99994;2.50000;150.00000;,
 149.99994;27.50000;200.00000;,
 370.00000;27.50000;0.00000;,
 365.77277;27.50000;42.91986;,
 365.77277;47.50000;42.91986;,
 370.00000;47.50000;0.00000;,
 353.25351;27.50000;84.19034;,
 353.25351;47.50000;84.19034;,
 332.92334;27.50000;122.22546;,
 332.92334;47.50000;122.22546;,
 305.56348;27.50000;155.56351;,
 305.56348;47.50000;155.56351;,
 272.22543;27.50000;182.92334;,
 272.22543;47.50000;182.92334;,
 234.19031;27.50000;203.25352;,
 234.19031;47.50000;203.25352;,
 192.91983;27.50000;215.77278;,
 192.91983;47.50000;215.77278;,
 149.99994;27.50000;220.00002;,
 149.99994;47.50000;220.00002;,
 149.99994;27.50000;220.00002;,
 192.91983;27.50000;215.77278;,
 234.19031;27.50000;203.25352;,
 272.22543;27.50000;182.92334;,
 305.56348;27.50000;155.56351;,
 332.92334;27.50000;122.22546;,
 353.25351;27.50000;84.19034;,
 365.77277;27.50000;42.91986;,
 370.00000;27.50000;0.00000;,
 390.00000;47.50000;-0.00002;,
 385.38849;47.50000;46.82165;,
 385.38849;-2.50000;46.82167;,
 390.00000;-2.50000;-0.00000;,
 371.73108;47.50000;91.84399;,
 371.73108;-2.50000;91.84400;,
 349.55273;47.50000;133.33684;,
 349.55273;-2.50000;133.33685;,
 319.70563;47.50000;169.70563;,
 319.70563;-2.50000;169.70564;,
 283.33685;47.50000;199.55272;,
 283.33685;-2.50000;199.55273;,
 241.84399;47.50000;221.73111;,
 241.84399;-2.50000;221.73112;,
 196.82162;47.50000;235.38847;,
 196.82162;-2.50000;235.38849;,
 149.99992;47.50000;240.00000;,
 149.99992;-2.50000;240.00002;,
 149.99994;47.50000;220.00002;,
 149.99992;47.50000;240.00000;,
 196.82162;47.50000;235.38847;,
 192.91983;47.50000;215.77278;,
 241.84399;47.50000;221.73111;,
 234.19031;47.50000;203.25352;,
 283.33685;47.50000;199.55272;,
 272.22543;47.50000;182.92334;,
 319.70563;47.50000;169.70563;,
 305.56348;47.50000;155.56351;,
 349.55273;47.50000;133.33684;,
 332.92334;47.50000;122.22546;,
 371.73108;47.50000;91.84399;,
 353.25351;47.50000;84.19034;,
 385.38849;47.50000;46.82165;,
 365.77277;47.50000;42.91986;,
 390.00000;47.50000;-0.00002;,
 370.00000;47.50000;0.00000;,
 297.11780;2.50000;-29.26353;,
 288.58194;2.50000;-57.40249;,
 274.72046;2.50000;-83.33553;,
 256.06601;2.50000;-106.06602;,
 233.33551;2.50000;-124.72045;,
 207.40250;2.50000;-138.58194;,
 179.26352;2.50000;-147.11780;,
 149.99994;2.50000;-150.00000;,
 346.15704;27.50000;-39.01804;,
 297.11780;2.50000;-29.26353;,
 334.77594;27.50000;-76.53666;,
 288.58194;2.50000;-57.40249;,
 316.29395;27.50000;-111.11404;,
 274.72046;2.50000;-83.33553;,
 291.42136;27.50000;-141.42136;,
 256.06601;2.50000;-106.06602;,
 261.11401;27.50000;-166.29393;,
 233.33551;2.50000;-124.72045;,
 226.53665;27.50000;-184.77592;,
 207.40250;2.50000;-138.58194;,
 189.01801;27.50000;-196.15706;,
 179.26352;2.50000;-147.11780;,
 149.99994;27.50000;-200.00000;,
 149.99994;2.50000;-150.00000;,
 365.77277;47.50000;-42.91986;,
 365.77277;27.50000;-42.91986;,
 353.25351;47.50000;-84.19034;,
 353.25351;27.50000;-84.19034;,
 332.92334;47.50000;-122.22546;,
 332.92334;27.50000;-122.22546;,
 305.56348;47.50000;-155.56351;,
 305.56348;27.50000;-155.56351;,
 272.22543;47.50000;-182.92334;,
 272.22543;27.50000;-182.92334;,
 234.19031;47.50000;-203.25352;,
 234.19031;27.50000;-203.25352;,
 192.91983;47.50000;-215.77278;,
 192.91983;27.50000;-215.77278;,
 149.99994;47.50000;-220.00002;,
 149.99994;27.50000;-220.00002;,
 192.91983;27.50000;-215.77278;,
 149.99994;27.50000;-220.00002;,
 234.19031;27.50000;-203.25352;,
 272.22543;27.50000;-182.92334;,
 305.56348;27.50000;-155.56351;,
 332.92334;27.50000;-122.22546;,
 353.25351;27.50000;-84.19034;,
 365.77277;27.50000;-42.91986;,
 385.38849;-2.50000;-46.82167;,
 385.38849;47.50000;-46.82165;,
 371.73108;-2.50000;-91.84400;,
 371.73108;47.50000;-91.84399;,
 349.55273;-2.50000;-133.33685;,
 349.55273;47.50000;-133.33684;,
 319.70563;-2.50000;-169.70564;,
 319.70563;47.50000;-169.70563;,
 283.33685;-2.50000;-199.55273;,
 283.33685;47.50000;-199.55272;,
 241.84399;-2.50000;-221.73112;,
 241.84399;47.50000;-221.73111;,
 196.82162;-2.50000;-235.38849;,
 196.82162;47.50000;-235.38847;,
 149.99992;-2.50000;-240.00002;,
 149.99992;47.50000;-240.00000;,
 149.99994;47.50000;-220.00002;,
 192.91983;47.50000;-215.77278;,
 196.82162;47.50000;-235.38847;,
 149.99992;47.50000;-240.00000;,
 234.19031;47.50000;-203.25352;,
 241.84399;47.50000;-221.73111;,
 272.22543;47.50000;-182.92334;,
 283.33685;47.50000;-199.55272;,
 305.56348;47.50000;-155.56351;,
 319.70563;47.50000;-169.70563;,
 332.92334;47.50000;-122.22546;,
 349.55273;47.50000;-133.33684;,
 353.25351;47.50000;-84.19034;,
 371.73108;47.50000;-91.84399;,
 365.77277;47.50000;-42.91986;,
 385.38849;47.50000;-46.82165;,
 40.00000;7.50000;-150.00000;,
 100.00000;2.50000;-150.00000;,
 40.00000;7.50000;-160.00000;,
 0.00000;27.50000;-200.00000;,
 40.00000;7.50000;-160.00000;,
 100.00000;2.50000;-150.00000;,
 0.00000;7.50000;-160.00000;,
 0.00000;7.50000;-150.00000;,
 0.00000;7.50000;-160.00000;,
 100.00000;2.50000;-130.00000;,
 40.00000;7.50000;-130.00000;,
 0.00000;7.50000;-130.00000;,
 100.00000;2.50000;-130.00000;,
 40.00000;7.50000;-130.00000;,
 40.00000;2.50000;-130.00000;,
 0.00000;7.50000;-130.00000;,
 0.00000;2.50000;-130.00000;,
 40.00000;2.50000;-130.00000;,
 0.00000;2.50000;-130.00000;,
 0.00000;2.50000;0.00000;,
 0.00000;27.50000;-220.00000;,
 0.00000;47.50000;-220.00000;,
 0.00000;27.50000;-220.00000;,
 0.00000;47.50000;-240.00000;,
 0.00000;47.50000;-220.00000;,
 -150.00000;2.50000;0.00000;,
 -300.00000;2.50000;-0.00001;,
 -297.11780;2.50000;29.26353;,
 -288.58194;2.50000;57.40249;,
 -274.72046;2.50000;83.33553;,
 -256.06601;2.50000;106.06602;,
 -233.33551;2.50000;124.72045;,
 -207.40250;2.50000;138.58194;,
 -179.26352;2.50000;147.11780;,
 -149.99994;2.50000;150.00000;,
 -300.00000;2.50000;-0.00001;,
 -350.00000;27.50000;-0.00002;,
 -346.15704;27.50000;39.01804;,
 -297.11780;2.50000;29.26353;,
 -334.77594;27.50000;76.53666;,
 -288.58194;2.50000;57.40249;,
 -316.29395;27.50000;111.11404;,
 -274.72046;2.50000;83.33553;,
 -291.42136;27.50000;141.42136;,
 -256.06601;2.50000;106.06602;,
 -261.11401;27.50000;166.29393;,
 -233.33551;2.50000;124.72045;,
 -226.53665;27.50000;184.77592;,
 -207.40250;2.50000;138.58194;,
 -189.01801;27.50000;196.15706;,
 -179.26352;2.50000;147.11780;,
 -149.99994;27.50000;200.00000;,
 -149.99994;2.50000;150.00000;,
 -370.00000;27.50000;0.00000;,
 -370.00000;47.50000;0.00000;,
 -365.77277;47.50000;42.91986;,
 -365.77277;27.50000;42.91986;,
 -353.25351;47.50000;84.19034;,
 -353.25351;27.50000;84.19034;,
 -332.92334;47.50000;122.22546;,
 -332.92334;27.50000;122.22546;,
 -305.56348;47.50000;155.56351;,
 -305.56348;27.50000;155.56351;,
 -272.22543;47.50000;182.92334;,
 -272.22543;27.50000;182.92334;,
 -234.19031;47.50000;203.25352;,
 -234.19031;27.50000;203.25352;,
 -192.91983;47.50000;215.77278;,
 -192.91983;27.50000;215.77278;,
 -149.99994;47.50000;220.00002;,
 -149.99994;27.50000;220.00002;,
 -192.91983;27.50000;215.77278;,
 -149.99994;27.50000;220.00002;,
 -234.19031;27.50000;203.25352;,
 -272.22543;27.50000;182.92334;,
 -305.56348;27.50000;155.56351;,
 -332.92334;27.50000;122.22546;,
 -353.25351;27.50000;84.19034;,
 -365.77277;27.50000;42.91986;,
 -370.00000;27.50000;0.00000;,
 -390.00000;47.50000;-0.00002;,
 -390.00000;-2.50000;0.00000;,
 -385.38849;-2.50000;46.82167;,
 -385.38849;47.50000;46.82165;,
 -371.73108;-2.50000;91.84400;,
 -371.73108;47.50000;91.84399;,
 -349.55273;-2.50000;133.33685;,
 -349.55273;47.50000;133.33684;,
 -319.70563;-2.50000;169.70564;,
 -319.70563;47.50000;169.70563;,
 -283.33685;-2.50000;199.55273;,
 -283.33685;47.50000;199.55272;,
 -241.84399;-2.50000;221.73112;,
 -241.84399;47.50000;221.73111;,
 -196.82162;-2.50000;235.38849;,
 -196.82162;47.50000;235.38847;,
 -149.99992;-2.50000;240.00002;,
 -149.99992;47.50000;240.00000;,
 -149.99994;47.50000;220.00002;,
 -192.91983;47.50000;215.77278;,
 -196.82162;47.50000;235.38847;,
 -149.99992;47.50000;240.00000;,
 -234.19031;47.50000;203.25352;,
 -241.84399;47.50000;221.73111;,
 -272.22543;47.50000;182.92334;,
 -283.33685;47.50000;199.55272;,
 -305.56348;47.50000;155.56351;,
 -319.70563;47.50000;169.70563;,
 -332.92334;47.50000;122.22546;,
 -349.55273;47.50000;133.33684;,
 -353.25351;47.50000;84.19034;,
 -371.73108;47.50000;91.84399;,
 -365.77277;47.50000;42.91986;,
 -385.38849;47.50000;46.82165;,
 -370.00000;47.50000;0.00000;,
 -390.00000;47.50000;-0.00002;,
 -297.11780;2.50000;-29.26353;,
 -288.58194;2.50000;-57.40249;,
 -274.72046;2.50000;-83.33553;,
 -256.06601;2.50000;-106.06602;,
 -233.33551;2.50000;-124.72045;,
 -207.40250;2.50000;-138.58194;,
 -179.26352;2.50000;-147.11780;,
 -149.99994;2.50000;-150.00000;,
 -297.11780;2.50000;-29.26353;,
 -346.15704;27.50000;-39.01804;,
 -288.58194;2.50000;-57.40249;,
 -334.77594;27.50000;-76.53666;,
 -274.72046;2.50000;-83.33553;,
 -316.29395;27.50000;-111.11404;,
 -256.06601;2.50000;-106.06602;,
 -291.42136;27.50000;-141.42136;,
 -233.33551;2.50000;-124.72045;,
 -261.11401;27.50000;-166.29393;,
 -207.40250;2.50000;-138.58194;,
 -226.53665;27.50000;-184.77592;,
 -179.26352;2.50000;-147.11780;,
 -189.01801;27.50000;-196.15706;,
 -149.99994;2.50000;-150.00000;,
 -149.99994;27.50000;-200.00000;,
 -365.77277;27.50000;-42.91986;,
 -365.77277;47.50000;-42.91986;,
 -353.25351;27.50000;-84.19034;,
 -353.25351;47.50000;-84.19034;,
 -332.92334;27.50000;-122.22546;,
 -332.92334;47.50000;-122.22546;,
 -305.56348;27.50000;-155.56351;,
 -305.56348;47.50000;-155.56351;,
 -272.22543;27.50000;-182.92334;,
 -272.22543;47.50000;-182.92334;,
 -234.19031;27.50000;-203.25352;,
 -234.19031;47.50000;-203.25352;,
 -192.91983;27.50000;-215.77278;,
 -192.91983;47.50000;-215.77278;,
 -149.99994;27.50000;-220.00002;,
 -149.99994;47.50000;-220.00002;,
 -149.99994;27.50000;-220.00002;,
 -192.91983;27.50000;-215.77278;,
 -234.19031;27.50000;-203.25352;,
 -272.22543;27.50000;-182.92334;,
 -305.56348;27.50000;-155.56351;,
 -332.92334;27.50000;-122.22546;,
 -353.25351;27.50000;-84.19034;,
 -365.77277;27.50000;-42.91986;,
 -385.38849;47.50000;-46.82165;,
 -385.38849;-2.50000;-46.82167;,
 -371.73108;47.50000;-91.84399;,
 -371.73108;-2.50000;-91.84400;,
 -349.55273;47.50000;-133.33684;,
 -349.55273;-2.50000;-133.33685;,
 -319.70563;47.50000;-169.70563;,
 -319.70563;-2.50000;-169.70564;,
 -283.33685;47.50000;-199.55272;,
 -283.33685;-2.50000;-199.55273;,
 -241.84399;47.50000;-221.73111;,
 -241.84399;-2.50000;-221.73112;,
 -196.82162;47.50000;-235.38847;,
 -196.82162;-2.50000;-235.38849;,
 -149.99992;47.50000;-240.00000;,
 -149.99992;-2.50000;-240.00002;,
 -149.99994;47.50000;-220.00002;,
 -149.99992;47.50000;-240.00000;,
 -196.82162;47.50000;-235.38847;,
 -192.91983;47.50000;-215.77278;,
 -241.84399;47.50000;-221.73111;,
 -234.19031;47.50000;-203.25352;,
 -283.33685;47.50000;-199.55272;,
 -272.22543;47.50000;-182.92334;,
 -319.70563;47.50000;-169.70563;,
 -305.56348;47.50000;-155.56351;,
 -349.55273;47.50000;-133.33684;,
 -332.92334;47.50000;-122.22546;,
 -371.73108;47.50000;-91.84399;,
 -353.25351;47.50000;-84.19034;,
 -385.38849;47.50000;-46.82165;,
 -365.77277;47.50000;-42.91986;,
 -40.00000;7.50000;-150.00000;,
 -40.00000;7.50000;-160.00000;,
 -100.00000;2.50000;-150.00000;,
 -40.00000;7.50000;-160.00000;,
 -100.00000;2.50000;-150.00000;,
 -100.00000;2.50000;-130.00000;,
 -40.00000;7.50000;-130.00000;,
 -100.00000;2.50000;-130.00000;,
 -40.00000;2.50000;-130.00000;,
 -40.00000;7.50000;-130.00000;,
 -40.00000;2.50000;-130.00000;,
 0.00000;47.50000;-240.00000;;
 
 239;
 3;0,1,2;,
 3;0,3,1;,
 3;0,4,3;,
 3;0,5,4;,
 3;0,6,5;,
 3;0,7,6;,
 3;0,8,7;,
 3;0,9,8;,
 4;10,11,12,13;,
 4;12,11,14,15;,
 4;14,16,17,15;,
 4;16,18,19,17;,
 4;18,20,21,19;,
 4;20,22,23,21;,
 4;22,24,25,23;,
 4;24,26,27,25;,
 4;28,29,30,31;,
 4;29,32,33,30;,
 4;32,34,35,33;,
 4;34,36,37,35;,
 4;36,38,39,37;,
 4;38,40,41,39;,
 4;40,42,43,41;,
 4;42,44,45,43;,
 4;27,46,47,25;,
 4;25,47,48,23;,
 4;23,48,49,21;,
 4;21,49,50,19;,
 4;19,50,51,17;,
 4;17,51,52,15;,
 4;15,52,53,12;,
 4;13,12,53,54;,
 4;55,56,57,58;,
 4;56,59,60,57;,
 4;59,61,62,60;,
 4;61,63,64,62;,
 4;63,65,66,64;,
 4;65,67,68,66;,
 4;67,69,70,68;,
 4;69,71,72,70;,
 4;73,74,75,76;,
 4;76,75,77,78;,
 4;78,77,79,80;,
 4;80,79,81,82;,
 4;82,81,83,84;,
 4;84,83,85,86;,
 4;86,85,87,88;,
 4;88,87,89,90;,
 3;0,2,91;,
 3;0,91,92;,
 3;0,92,93;,
 3;0,93,94;,
 3;0,94,95;,
 3;0,95,96;,
 3;0,96,97;,
 3;0,97,98;,
 4;10,13,99,100;,
 4;99,101,102,100;,
 4;102,101,103,104;,
 4;104,103,105,106;,
 4;106,105,107,108;,
 4;108,107,109,110;,
 4;110,109,111,112;,
 4;112,111,113,114;,
 4;28,31,115,116;,
 4;116,115,117,118;,
 4;118,117,119,120;,
 4;120,119,121,122;,
 4;122,121,123,124;,
 4;124,123,125,126;,
 4;126,125,127,128;,
 4;128,127,129,130;,
 4;113,111,131,132;,
 4;111,109,133,131;,
 4;109,107,134,133;,
 4;107,105,135,134;,
 4;105,103,136,135;,
 4;103,101,137,136;,
 4;101,99,138,137;,
 4;13,54,138,99;,
 4;55,58,139,140;,
 4;140,139,141,142;,
 4;142,141,143,144;,
 4;144,143,145,146;,
 4;146,145,147,148;,
 4;148,147,149,150;,
 4;150,149,151,152;,
 4;152,151,153,154;,
 4;155,156,157,158;,
 4;156,159,160,157;,
 4;159,161,162,160;,
 4;161,163,164,162;,
 4;163,165,166,164;,
 4;165,167,168,166;,
 4;167,169,170,168;,
 4;169,90,89,170;,
 3;171,172,173;,
 3;174,175,113;,
 3;113,175,176;,
 3;176,114,113;,
 4;171,173,177,178;,
 3;174,179,175;,
 4;180,172,171,181;,
 4;181,171,178,182;,
 3;183,184,185;,
 4;185,184,186,187;,
 3;98,172,180;,
 3;180,0,98;,
 3;188,189,190;,
 3;180,188,190;,
 3;190,0,180;,
 4;174,113,132,191;,
 4;130,129,192,193;,
 4;155,158,194,195;,
 3;196,197,198;,
 3;196,198,199;,
 3;196,199,200;,
 3;196,200,201;,
 3;196,201,202;,
 3;196,202,203;,
 3;196,203,204;,
 3;196,204,205;,
 4;206,207,208,209;,
 4;208,210,211,209;,
 4;211,210,212,213;,
 4;213,212,214,215;,
 4;215,214,216,217;,
 4;217,216,218,219;,
 4;219,218,220,221;,
 4;221,220,222,223;,
 4;224,225,226,227;,
 4;227,226,228,229;,
 4;229,228,230,231;,
 4;231,230,232,233;,
 4;233,232,234,235;,
 4;235,234,236,237;,
 4;237,236,238,239;,
 4;239,238,240,241;,
 4;222,220,242,243;,
 4;220,218,244,242;,
 4;218,216,245,244;,
 4;216,214,246,245;,
 4;214,212,247,246;,
 4;212,210,248,247;,
 4;210,208,249,248;,
 4;207,250,249,208;,
 4;251,252,253,254;,
 4;254,253,255,256;,
 4;256,255,257,258;,
 4;258,257,259,260;,
 4;260,259,261,262;,
 4;262,261,263,264;,
 4;264,263,265,266;,
 4;266,265,267,268;,
 4;269,270,271,272;,
 4;270,273,274,271;,
 4;273,275,276,274;,
 4;275,277,278,276;,
 4;277,279,280,278;,
 4;279,281,282,280;,
 4;281,283,284,282;,
 4;283,285,286,284;,
 3;196,287,197;,
 3;196,288,287;,
 3;196,289,288;,
 3;196,290,289;,
 3;196,291,290;,
 3;196,292,291;,
 3;196,293,292;,
 3;196,294,293;,
 4;206,295,296,207;,
 4;296,295,297,298;,
 4;297,299,300,298;,
 4;299,301,302,300;,
 4;301,303,304,302;,
 4;303,305,306,304;,
 4;305,307,308,306;,
 4;307,309,310,308;,
 4;224,311,312,225;,
 4;311,313,314,312;,
 4;313,315,316,314;,
 4;315,317,318,316;,
 4;317,319,320,318;,
 4;319,321,322,320;,
 4;321,323,324,322;,
 4;323,325,326,324;,
 4;310,327,328,308;,
 4;308,328,329,306;,
 4;306,329,330,304;,
 4;304,330,331,302;,
 4;302,331,332,300;,
 4;300,332,333,298;,
 4;298,333,334,296;,
 4;207,296,334,250;,
 4;251,335,336,252;,
 4;335,337,338,336;,
 4;337,339,340,338;,
 4;339,341,342,340;,
 4;341,343,344,342;,
 4;343,345,346,344;,
 4;345,347,348,346;,
 4;347,349,350,348;,
 4;351,352,353,354;,
 4;354,353,355,356;,
 4;356,355,357,358;,
 4;358,357,359,360;,
 4;360,359,361,362;,
 4;362,361,363,364;,
 4;364,363,365,366;,
 4;366,365,286,285;,
 3;367,368,369;,
 3;174,310,370;,
 3;310,371,370;,
 3;371,310,309;,
 4;367,178,177,368;,
 3;174,370,179;,
 4;372,373,367,369;,
 4;373,182,178,367;,
 3;374,375,376;,
 4;375,187,186,376;,
 3;294,372,369;,
 3;372,294,196;,
 3;377,190,189;,
 3;372,190,377;,
 3;190,372,196;,
 4;174,191,327,310;,
 4;325,193,192,326;,
 4;351,195,194,352;,
 4;222,243,46,27;,
 4;241,240,45,44;,
 4;27,26,223,222;,
 3;190,196,205;,
 3;190,9,0;,
 3;190,205,9;,
 3;154,153,378;,
 3;378,350,349;,
 3;350,378,153;,
 4;269,272,74,73;,
 4;268,267,72,71;;
 
 MeshMaterialList {
  2;
  239;
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  0,
  0,
  0,
  1,
  0,
  1,
  1,
  0,
  0,
  1,
  1,
  1,
  1,
  1,
  0,
  0,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  0,
  0,
  0,
  1,
  0,
  1,
  1,
  0,
  0,
  1,
  1,
  1,
  1,
  1,
  0,
  0,
  1,
  0,
  0,
  0,
  1,
  1,
  1,
  0,
  0,
  0,
  1,
  0;;
  Material {
   0.558431;0.505098;0.363922;1.000000;;
   5.000000;
   0.000000;0.000000;0.000000;;
   0.000000;0.000000;0.000000;;
   TextureFilename {
    "stone.png";
   }
  }
  Material {
   0.800000;0.800000;0.800000;1.000000;;
   5.000000;
   0.000000;0.000000;0.000000;;
   0.000000;0.000000;0.000000;;
   TextureFilename {
    "grass.png";
   }
  }
 }
 MeshNormals {
  110;
  0.000000;1.000000;0.000000;,
  -0.446781;0.893561;-0.044004;,
  -0.429611;0.893561;-0.130321;,
  -0.395932;0.893561;-0.211630;,
  -0.347037;0.893561;-0.284806;,
  -0.284806;0.893561;-0.347037;,
  -0.211630;0.893561;-0.395932;,
  -0.130321;0.893561;-0.429611;,
  -0.044004;0.893561;-0.446780;,
  -0.995185;0.000000;-0.098017;,
  -0.956940;0.000000;-0.290285;,
  -0.881921;0.000000;-0.471396;,
  -0.773010;0.000000;-0.634394;,
  -0.634393;0.000000;-0.773010;,
  -0.471397;0.000000;-0.881921;,
  -0.290285;0.000000;-0.956940;,
  -0.098017;0.000000;-0.995185;,
  0.995185;0.000000;0.098017;,
  0.956940;0.000000;0.290285;,
  0.881922;0.000000;0.471396;,
  0.773010;0.000000;0.634393;,
  0.634393;0.000000;0.773010;,
  0.471397;0.000000;0.881921;,
  0.290284;0.000000;0.956940;,
  0.098017;0.000000;0.995185;,
  -0.446780;0.893561;0.044004;,
  -0.429611;0.893561;0.130321;,
  -0.395932;0.893561;0.211630;,
  -0.347037;0.893561;0.284806;,
  -0.284806;0.893561;0.347037;,
  -0.211630;0.893561;0.395932;,
  -0.130321;0.893561;0.429611;,
  -0.044004;0.893561;0.446780;,
  -0.995185;0.000000;0.098017;,
  -0.956940;0.000000;0.290285;,
  -0.881921;0.000000;0.471396;,
  -0.773010;0.000000;0.634394;,
  -0.634393;0.000000;0.773010;,
  -0.471397;0.000000;0.881921;,
  -0.290285;0.000000;0.956940;,
  -0.098017;0.000000;0.995185;,
  0.995185;0.000000;-0.098017;,
  0.956940;0.000000;-0.290285;,
  0.881922;0.000000;-0.471396;,
  0.773010;0.000000;-0.634393;,
  0.634393;0.000000;-0.773010;,
  0.471397;0.000000;-0.881921;,
  0.290284;0.000000;-0.956940;,
  0.098017;0.000000;-0.995185;,
  0.083045;0.996546;0.000000;,
  0.000000;0.894427;0.447214;,
  0.000000;0.000000;1.000000;,
  0.083045;0.996546;0.000000;,
  0.000000;0.000000;1.000000;,
  0.446781;0.893561;-0.044004;,
  0.429611;0.893561;-0.130321;,
  0.395932;0.893561;-0.211630;,
  0.347037;0.893561;-0.284806;,
  0.284806;0.893561;-0.347037;,
  0.211630;0.893561;-0.395932;,
  0.130321;0.893561;-0.429611;,
  0.044004;0.893561;-0.446780;,
  0.995185;0.000000;-0.098017;,
  0.956940;0.000000;-0.290285;,
  0.881921;0.000000;-0.471396;,
  0.773010;0.000000;-0.634394;,
  0.634393;0.000000;-0.773010;,
  0.471397;0.000000;-0.881921;,
  0.290285;0.000000;-0.956940;,
  0.098017;0.000000;-0.995185;,
  -0.995185;0.000000;0.098017;,
  -0.956940;0.000000;0.290285;,
  -0.881922;0.000000;0.471396;,
  -0.773010;0.000000;0.634393;,
  -0.634393;0.000000;0.773010;,
  -0.471397;0.000000;0.881921;,
  -0.290284;0.000000;0.956940;,
  -0.098017;0.000000;0.995185;,
  0.446780;0.893561;0.044004;,
  0.429611;0.893561;0.130321;,
  0.395932;0.893561;0.211630;,
  0.347037;0.893561;0.284806;,
  0.284806;0.893561;0.347037;,
  0.211630;0.893561;0.395932;,
  0.130321;0.893561;0.429611;,
  0.044004;0.893561;0.446780;,
  0.995185;0.000000;0.098017;,
  0.956940;0.000000;0.290285;,
  0.881921;0.000000;0.471396;,
  0.773010;0.000000;0.634394;,
  0.634393;0.000000;0.773010;,
  0.471397;0.000000;0.881921;,
  0.290285;0.000000;0.956940;,
  0.098017;0.000000;0.995185;,
  -0.995185;0.000000;-0.098017;,
  -0.956940;0.000000;-0.290285;,
  -0.881922;0.000000;-0.471396;,
  -0.773010;0.000000;-0.634393;,
  -0.634393;0.000000;-0.773010;,
  -0.471397;0.000000;-0.881921;,
  -0.290284;0.000000;-0.956940;,
  -0.098017;0.000000;-0.995185;,
  -0.083045;0.996546;0.000000;,
  -0.083045;0.996546;0.000000;,
  0.000000;1.000000;0.000000;,
  -0.000000;0.000000;1.000000;,
  0.000000;0.000000;-1.000000;,
  0.000000;0.894427;-0.447214;,
  0.000000;0.000000;-1.000000;,
  0.000000;0.000000;1.000000;;
  239;
  3;0,0,0;,
  3;0,0,0;,
  3;0,0,0;,
  3;0,0,0;,
  3;0,0,0;,
  3;0,0,0;,
  3;0,0,0;,
  3;0,0,0;,
  4;1,1,1,1;,
  4;2,2,2,2;,
  4;3,3,3,3;,
  4;4,4,4,4;,
  4;5,5,5,5;,
  4;6,6,6,6;,
  4;7,7,7,7;,
  4;8,8,8,8;,
  4;9,9,9,9;,
  4;10,10,10,10;,
  4;11,11,11,11;,
  4;12,12,12,12;,
  4;13,13,13,13;,
  4;14,14,14,14;,
  4;15,15,15,15;,
  4;16,16,16,16;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;17,17,17,17;,
  4;18,18,18,18;,
  4;19,19,19,19;,
  4;20,20,20,20;,
  4;21,21,21,21;,
  4;22,22,22,22;,
  4;23,23,23,23;,
  4;24,24,24,24;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  3;0,0,0;,
  3;0,0,0;,
  3;0,0,0;,
  3;0,0,0;,
  3;0,0,0;,
  3;0,0,0;,
  3;0,0,0;,
  3;0,0,0;,
  4;25,25,25,25;,
  4;26,26,26,26;,
  4;27,27,27,27;,
  4;28,28,28,28;,
  4;29,29,29,29;,
  4;30,30,30,30;,
  4;31,31,31,31;,
  4;32,32,32,32;,
  4;33,33,33,33;,
  4;34,34,34,34;,
  4;35,35,35,35;,
  4;36,36,36,36;,
  4;37,37,37,37;,
  4;38,38,38,38;,
  4;39,39,39,39;,
  4;40,40,40,40;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;41,41,41,41;,
  4;42,42,42,42;,
  4;43,43,43,43;,
  4;44,44,44,44;,
  4;45,45,45,45;,
  4;46,46,46,46;,
  4;47,47,47,47;,
  4;48,48,48,48;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  3;49,49,49;,
  3;50,50,50;,
  3;50,50,50;,
  3;50,50,50;,
  4;0,0,0,0;,
  3;50,50,50;,
  4;52,52,52,52;,
  4;0,0,0,0;,
  3;53,53,53;,
  4;53,53,53,53;,
  3;0,0,0;,
  3;104,104,104;,
  3;0,0,0;,
  3;104,104,104;,
  3;0,0,0;,
  4;0,0,0,0;,
  4;51,51,51,51;,
  4;0,0,0,0;,
  3;0,0,0;,
  3;0,0,0;,
  3;0,0,0;,
  3;0,0,0;,
  3;0,0,0;,
  3;0,0,0;,
  3;0,0,0;,
  3;0,0,0;,
  4;54,54,54,54;,
  4;55,55,55,55;,
  4;56,56,56,56;,
  4;57,57,57,57;,
  4;58,58,58,58;,
  4;59,59,59,59;,
  4;60,60,60,60;,
  4;61,61,61,61;,
  4;62,62,62,62;,
  4;63,63,63,63;,
  4;64,64,64,64;,
  4;65,65,65,65;,
  4;66,66,66,66;,
  4;67,67,67,67;,
  4;68,68,68,68;,
  4;69,69,69,69;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;70,70,70,70;,
  4;71,71,71,71;,
  4;72,72,72,72;,
  4;73,73,73,73;,
  4;74,74,74,74;,
  4;75,75,75,75;,
  4;76,76,76,76;,
  4;77,77,77,77;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  3;0,0,0;,
  3;0,0,0;,
  3;0,0,0;,
  3;0,0,0;,
  3;0,0,0;,
  3;0,0,0;,
  3;0,0,0;,
  3;0,0,0;,
  4;78,78,78,78;,
  4;79,79,79,79;,
  4;80,80,80,80;,
  4;81,81,81,81;,
  4;82,82,82,82;,
  4;83,83,83,83;,
  4;84,84,84,84;,
  4;85,85,85,85;,
  4;86,86,86,86;,
  4;87,87,87,87;,
  4;88,88,88,88;,
  4;89,89,89,89;,
  4;90,90,90,90;,
  4;91,91,91,91;,
  4;92,92,92,92;,
  4;93,93,93,93;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;94,94,94,94;,
  4;95,95,95,95;,
  4;96,96,96,96;,
  4;97,97,97,97;,
  4;98,98,98,98;,
  4;99,99,99,99;,
  4;100,100,100,100;,
  4;101,101,101,101;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  3;102,102,102;,
  3;50,50,50;,
  3;50,50,50;,
  3;50,50,50;,
  4;0,0,0,0;,
  3;50,50,50;,
  4;103,103,103,103;,
  4;0,0,0,0;,
  3;53,53,53;,
  4;53,53,53,53;,
  3;0,0,0;,
  3;104,104,104;,
  3;0,0,0;,
  3;104,104,104;,
  3;0,0,0;,
  4;0,0,0,0;,
  4;105,105,105,105;,
  4;0,0,0,0;,
  4;0,0,0,0;,
  4;106,106,106,106;,
  4;107,107,107,107;,
  3;0,0,0;,
  3;0,0,0;,
  3;0,0,0;,
  3;108,108,108;,
  3;108,108,108;,
  3;108,108,108;,
  4;0,0,0,0;,
  4;109,109,109,109;;
 }
 MeshTextureCoords {
  379;
  3.807084;0.518104;,
  6.738036;-0.064898;,
  6.795456;0.518105;,
  6.567980;-0.625496;,
  6.291825;-1.142146;,
  5.920182;-1.594994;,
  5.467334;-1.966637;,
  4.950685;-2.242792;,
  4.390086;-2.412848;,
  3.807082;-2.470268;,
  8.195311;1.038337;,
  8.111377;0.186146;,
  9.539461;-0.097917;,
  9.651373;1.038338;,
  7.862802;-0.633296;,
  9.208030;-1.190506;,
  7.459138;-1.388498;,
  8.669810;-2.197443;,
  6.915897;-2.050438;,
  7.945489;-3.080030;,
  6.253957;-2.593679;,
  7.062902;-3.804351;,
  5.498755;-2.997343;,
  6.055965;-4.342570;,
  4.679313;-3.245918;,
  4.963376;-4.674003;,
  3.827121;-3.329851;,
  3.827121;-4.785914;,
  10.500768;3.309277;,
  10.428575;3.309277;,
  10.428575;2.967718;,
  10.500768;2.967718;,
  10.214772;3.309277;,
  10.214772;2.967718;,
  9.867573;3.309277;,
  9.867573;2.967718;,
  9.400322;3.309277;,
  9.400322;2.967718;,
  8.830976;3.309277;,
  8.830976;2.967718;,
  8.181413;3.309277;,
  8.181413;2.967718;,
  7.476597;3.309277;,
  7.476597;2.967718;,
  6.743612;3.309277;,
  6.743612;2.967718;,
  3.827121;-5.368340;,
  5.077002;-5.245237;,
  6.278850;-4.880661;,
  7.386481;-4.288620;,
  8.357326;-3.491868;,
  9.154079;-2.521022;,
  9.746119;-1.413391;,
  10.110696;-0.211543;,
  10.233798;1.038337;,
  10.842327;2.967718;,
  10.763572;2.967718;,
  10.763572;4.333955;,
  10.842327;4.333955;,
  10.530331;2.967718;,
  10.530331;4.333955;,
  10.151569;2.967718;,
  10.151569;4.333955;,
  9.641842;2.967718;,
  9.641842;4.333955;,
  9.020737;2.967718;,
  9.020737;4.333955;,
  8.312122;2.967718;,
  8.312122;4.333955;,
  7.543232;2.967718;,
  7.543232;4.333955;,
  6.743612;2.967718;,
  6.743612;4.333955;,
  6.256571;-8.818709;,
  6.256570;-9.456233;,
  7.749066;-9.309235;,
  7.624692;-8.683962;,
  9.184206;-8.873889;,
  8.940236;-8.284896;,
  10.506839;-8.166928;,
  10.152650;-7.636848;,
  11.666136;-7.215518;,
  11.215339;-6.764721;,
  12.617546;-6.056220;,
  12.087465;-5.702032;,
  13.324507;-4.733587;,
  12.735513;-4.489619;,
  13.759852;-3.298449;,
  13.134579;-3.174074;,
  13.906850;-1.805954;,
  13.269326;-1.805954;,
  6.738036;1.101107;,
  6.567980;1.661705;,
  6.291825;2.178355;,
  5.920182;2.631203;,
  5.467334;3.002846;,
  4.950685;3.279001;,
  4.390086;3.449056;,
  3.807082;3.506477;,
  9.539461;2.174592;,
  8.111377;1.890528;,
  9.208030;3.267181;,
  7.862802;2.709970;,
  8.669810;4.274117;,
  7.459138;3.465172;,
  7.945489;5.156704;,
  6.915897;4.127112;,
  7.062902;5.881025;,
  6.253957;4.670353;,
  6.055965;6.419244;,
  5.498755;5.074017;,
  4.963376;6.750677;,
  4.679313;5.322592;,
  3.827121;6.862588;,
  3.827121;5.406526;,
  10.428575;2.967718;,
  10.428575;3.309277;,
  10.214772;2.967718;,
  10.214772;3.309277;,
  9.867573;2.967718;,
  9.867573;3.309277;,
  9.400322;2.967718;,
  9.400322;3.309277;,
  8.830976;2.967718;,
  8.830976;3.309277;,
  8.181413;2.967718;,
  8.181413;3.309277;,
  7.476597;2.967718;,
  7.476597;3.309277;,
  6.743612;2.967718;,
  6.743612;3.309277;,
  5.077002;7.321911;,
  3.827121;7.445014;,
  6.278850;6.957335;,
  7.386481;6.365294;,
  8.357326;5.568542;,
  9.154079;4.597696;,
  9.746119;3.490065;,
  10.110696;2.288217;,
  10.763572;4.333955;,
  10.763572;2.967718;,
  10.530331;4.333955;,
  10.530331;2.967718;,
  10.151569;4.333955;,
  10.151569;2.967718;,
  9.641842;4.333955;,
  9.641842;2.967718;,
  9.020737;4.333955;,
  9.020737;2.967718;,
  8.312122;4.333955;,
  8.312122;2.967718;,
  7.543232;4.333955;,
  7.543232;2.967718;,
  6.743612;4.333955;,
  6.743612;2.967718;,
  6.256571;5.206801;,
  7.624692;5.072053;,
  7.749066;5.697326;,
  6.256570;5.844324;,
  8.940236;4.672987;,
  9.184206;5.261982;,
  10.152650;4.024939;,
  10.506839;4.555020;,
  11.215339;3.152813;,
  11.666136;3.603609;,
  12.087465;2.090124;,
  12.617546;2.444312;,
  12.735513;0.877710;,
  13.324507;1.121679;,
  13.134579;-0.437834;,
  13.759852;-0.313460;,
  1.615611;3.506477;,
  2.810960;3.506477;,
  1.615611;3.705702;,
  -0.541066;6.862588;,
  0.623784;5.697738;,
  2.371059;5.406526;,
  0.818711;3.705702;,
  0.818711;3.506477;,
  -0.541066;5.697738;,
  2.810960;3.108027;,
  1.615611;3.108027;,
  0.818711;3.108027;,
  5.889714;4.163176;,
  4.865036;3.992396;,
  4.865036;4.163176;,
  4.181917;3.992396;,
  4.181917;4.163176;,
  1.615611;3.108027;,
  0.818711;3.108027;,
  0.818711;0.518104;,
  -0.541066;7.445013;,
  4.181917;2.967718;,
  4.181917;3.309277;,
  1.475149;5.844324;,
  1.475149;5.206801;,
  -2.169661;0.518104;,
  -5.158034;0.518105;,
  -5.100613;-0.064898;,
  -4.930558;-0.625496;,
  -4.654403;-1.142146;,
  -4.282759;-1.594994;,
  -3.829912;-1.966637;,
  -3.313262;-2.242792;,
  -2.752663;-2.412848;,
  -2.169660;-2.470268;,
  -9.277444;1.038337;,
  -10.733506;1.038338;,
  -10.621594;-0.097917;,
  -9.193510;0.186146;,
  -10.290162;-1.190506;,
  -8.944935;-0.633296;,
  -9.751943;-2.197443;,
  -8.541271;-1.388498;,
  -9.027622;-3.080030;,
  -7.998030;-2.050438;,
  -8.145034;-3.804351;,
  -7.336089;-2.593679;,
  -7.138098;-4.342570;,
  -6.580887;-2.997343;,
  -6.045508;-4.674003;,
  -5.761445;-3.245918;,
  -4.909252;-4.785914;,
  -4.909252;-3.329851;,
  -2.136934;3.309277;,
  -2.136934;2.967718;,
  -2.064741;2.967718;,
  -2.064741;3.309277;,
  -1.850938;2.967718;,
  -1.850938;3.309277;,
  -1.503740;2.967718;,
  -1.503740;3.309277;,
  -1.036489;2.967718;,
  -1.036489;3.309277;,
  -0.467142;2.967718;,
  -0.467142;3.309277;,
  0.182421;2.967718;,
  0.182421;3.309277;,
  0.887237;2.967718;,
  0.887237;3.309277;,
  1.620222;2.967718;,
  1.620222;3.309277;,
  -6.159134;-5.245237;,
  -4.909252;-5.368340;,
  -7.360981;-4.880661;,
  -8.468613;-4.288620;,
  -9.439459;-3.491868;,
  -10.236213;-2.521022;,
  -10.828253;-1.413391;,
  -11.192829;-0.211543;,
  -11.315930;1.038337;,
  -2.478493;2.967718;,
  -2.478493;4.333955;,
  -2.399738;4.333955;,
  -2.399738;2.967718;,
  -2.166497;4.333955;,
  -2.166497;2.967718;,
  -1.787736;4.333955;,
  -1.787736;2.967718;,
  -1.278008;4.333955;,
  -1.278008;2.967718;,
  -0.656903;4.333955;,
  -0.656903;2.967718;,
  0.051711;4.333955;,
  0.051711;2.967718;,
  0.820602;4.333955;,
  0.820602;2.967718;,
  1.620222;4.333955;,
  1.620222;2.967718;,
  -3.306273;-8.818709;,
  -4.674394;-8.683962;,
  -4.798768;-9.309235;,
  -3.306273;-9.456233;,
  -5.989939;-8.284896;,
  -6.233909;-8.873889;,
  -7.202352;-7.636848;,
  -7.556541;-8.166928;,
  -8.265041;-6.764721;,
  -8.715838;-7.215518;,
  -9.137168;-5.702032;,
  -9.667250;-6.056220;,
  -9.785216;-4.489619;,
  -10.374210;-4.733587;,
  -10.184282;-3.174074;,
  -10.809556;-3.298449;,
  -10.319031;-1.805954;,
  -10.956553;-1.805954;,
  -5.100613;1.101107;,
  -4.930558;1.661705;,
  -4.654403;2.178355;,
  -4.282759;2.631203;,
  -3.829912;3.002846;,
  -3.313262;3.279001;,
  -2.752663;3.449056;,
  -2.169660;3.506477;,
  -9.193510;1.890528;,
  -10.621594;2.174592;,
  -8.944935;2.709970;,
  -10.290162;3.267181;,
  -8.541271;3.465172;,
  -9.751943;4.274117;,
  -7.998030;4.127112;,
  -9.027622;5.156704;,
  -7.336089;4.670353;,
  -8.145034;5.881025;,
  -6.580887;5.074017;,
  -7.138098;6.419244;,
  -5.761445;5.322592;,
  -6.045508;6.750677;,
  -4.909252;5.406526;,
  -4.909252;6.862588;,
  -2.064741;3.309277;,
  -2.064741;2.967718;,
  -1.850938;3.309277;,
  -1.850938;2.967718;,
  -1.503740;3.309277;,
  -1.503740;2.967718;,
  -1.036489;3.309277;,
  -1.036489;2.967718;,
  -0.467142;3.309277;,
  -0.467142;2.967718;,
  0.182421;3.309277;,
  0.182421;2.967718;,
  0.887237;3.309277;,
  0.887237;2.967718;,
  1.620222;3.309277;,
  1.620222;2.967718;,
  -4.909252;7.445014;,
  -6.159134;7.321911;,
  -7.360981;6.957335;,
  -8.468613;6.365294;,
  -9.439459;5.568542;,
  -10.236213;4.597696;,
  -10.828253;3.490065;,
  -11.192829;2.288217;,
  -2.399738;2.967718;,
  -2.399738;4.333955;,
  -2.166497;2.967718;,
  -2.166497;4.333955;,
  -1.787736;2.967718;,
  -1.787736;4.333955;,
  -1.278008;2.967718;,
  -1.278008;4.333955;,
  -0.656903;2.967718;,
  -0.656903;4.333955;,
  0.051711;2.967718;,
  0.051711;4.333955;,
  0.820602;2.967718;,
  0.820602;4.333955;,
  1.620222;2.967718;,
  1.620222;4.333955;,
  -3.306273;5.206801;,
  -3.306273;5.844324;,
  -4.798768;5.697326;,
  -4.674394;5.072053;,
  -6.233909;5.261982;,
  -5.989939;4.672987;,
  -7.556541;4.555020;,
  -7.202352;4.024939;,
  -8.715838;3.603609;,
  -8.265041;3.152813;,
  -9.667250;2.444312;,
  -9.137168;2.090124;,
  -10.374210;1.121679;,
  -9.785216;0.877710;,
  -10.809556;-0.313460;,
  -10.184282;-0.437834;,
  0.021812;3.506477;,
  0.021812;3.705702;,
  -1.173537;3.506477;,
  -1.705916;5.697738;,
  -3.453192;5.406526;,
  -1.173537;3.108027;,
  0.021812;3.108027;,
  2.474119;4.163176;,
  3.498798;4.163176;,
  3.498798;3.992396;,
  0.021812;3.108027;,
  4.181917;2.967718;;
 }
}
