from xml.dom import minidom
import plotly.graph_objects as go


def GetName(node):
    names = [tag.attributes['v'].value for tag in node.getElementsByTagName('tag') if tag.attributes['k'].value == "name"]

    if len(names) > 0:
        return node.attributes['id'].value + '. ' + names[0]
    return node.attributes['id'].value + '.'


def IsStop(node):
    if node.attributes['visible'].value == 'false':
        return False
    tags = [tag for tag in node.getElementsByTagName('tag') if
            tag.attributes['k'].value == "public_transport" and tag.attributes['v'].value == "stop_position"]
    return len(tags) > 0


def IsPath(node, type):
    if node.attributes['visible'].value == 'false':
        return False
    tags = [tag for tag in node.getElementsByTagName('tag') if
            tag.attributes['k'].value == "highway" and tag.attributes['v'].value == type]
    return len(tags) > 0


def GetPathNodes(nodes, node):
    ids = [nd.attributes['ref'].value for nd in node.getElementsByTagName('nd') if 'ref' in nd.attributes]
    path_nodes = {node.attributes['id'].value: node for node in nodes if node.attributes['id'].value in ids}

    return [path_nodes[id] for id in ids]


def GetLatLon(node):
    return float(node.attributes['lon'].value), float(node.attributes['lat'].value)


def AddPath(data, nodes, type, marker):
    print(f"Parsing ways {type} ...")
    poss = [([GetLatLon(node) for node in GetPathNodes(nodes, path)], GetName(path)) for path in ways if IsPath(path, type)]
    first = True
    for way, name in poss:
        x, y = [pos[0] for pos in way], [pos[1] for pos in way]
        data.append(go.Scatter(x=x, y=y, mode="lines", marker=marker, text=name, name=type, legendgroup=type, showlegend=first))
        first = False


print("Parsing document ...")
xmldoc = minidom.parse('poruba.osm')

data = []
print("Parsing nodes ...")
nodes = xmldoc.getElementsByTagName('node')
print("Parsing ways ...")
ways = xmldoc.getElementsByTagName('way')

stopMarker = go.scatter.Marker(color='red')
resMarker = go.scatter.Marker(color='green')
secMarker = go.scatter.Marker(color='orange')
prmMarker = go.scatter.Marker(color='blue')
cycleMarker = go.scatter.Marker(color='darkgreen')
footMarker = go.scatter.Marker(color='gray')

AddPath(data, nodes, "footway", footMarker)
AddPath(data, nodes, "residential", resMarker)
AddPath(data, nodes, "cycleway", cycleMarker)
AddPath(data, nodes, "secondary", secMarker)
AddPath(data, nodes, "primary", prmMarker)

print("Stops ...")
poss = [(GetLatLon(stop), GetName(stop)) for stop in [stop for stop in nodes if IsStop(stop)]]
x, y = [pos[0][0] for pos in poss], [pos[0][1] for pos in poss]
names = [pos[1] for pos in poss]
data.append(go.Scatter(x=x, y=y, mode='markers', marker=stopMarker, marker_size=10, text=names, name="stops"))

print("Done")
fig = go.Figure(data=data)
fig.show()
