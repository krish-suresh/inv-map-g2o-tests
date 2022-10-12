fname = 'map.json'; 
fid = fopen(fname); 
raw = fread(fid,inf); 
str = char(raw'); 
fclose(fid); 
val = jsondecode(str);
data = val.tag_vertices;
X = zeros(1,length(data));
Y = zeros(1, length(data));
Z = zeros(1, length(data));
for i=1:length(data)
    d = data(i);
    X(i) = d.translation.x;
    Y(i) = d.translation.y;
    Z(i) = d.translation.z;
end