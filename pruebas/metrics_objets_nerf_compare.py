
import open3d as o3d
import numpy as np
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go

f = open("metrics.txt", "w")
fig = go.Figure()
counter = 0
for i in ["Coche","Edificio","Estatua"]:

    metodos = []
    group = []
    points = []
    values = []
    means = []
    methods = ["NeRF"]
    for a in range(len(methods)):
        or_mesh = o3d.io.read_triangle_mesh(str(i)+"_or.obj")

        mesh = o3d.io.read_triangle_mesh(str(i)+"_"+methods[a]+".obj")
        z_rot = np.pi
        x_rot = -np.pi/2
        y_rot=0
        z_translate = 0
        x_translate = 0
        y_translate = 0

        if methods[a]!="Poisson_Surface":
            z_rot+=np.pi

        if i=="Estatua":
            or_mesh=or_mesh.scale(0.1, center=mesh.get_center())
            x_translate=-10
            y_translate=20
            z_translate=30
            if methods[a]=="Poisson_Surface":
                y_translate = -240
                x_translate=40
            if methods[a]=="Ball_Pivoting":
                z_translate=10
            if methods[a]=="NeRF":
                mesh=mesh.scale(150, center=mesh.get_center())
                x_rot+= np.pi/2
                y_rot+= np.pi/2+0.2
                z_translate+=230
                y_translate-=200
                x_translate-=30

        elif i=="Coche":
            z_rot-=0.9
            z_translate=100
            if methods[a]=="Poisson_Surface":
                z_rot-=0
            if "NeRF" in methods[a]:
                mesh=mesh.scale(150, center=mesh.get_center())
                x_rot+= np.pi/2
                z_translate-=50
                z_rot+=0.8
                

        elif i=="Edificio":
            z_translate=800
            y_translate=-150
            x_translate=20
            z_rot+=np.pi/2-0.2
            
            if methods[a]=="NeRF":
                mesh=mesh.scale(300, center=mesh.get_center())
                y_rot+= np.pi/2
                z_translate+=200
                x_translate+=200

        R = mesh.get_rotation_matrix_from_xyz((x_rot,y_rot,z_rot))
        mesh = mesh.rotate(R, center=mesh.get_center())
        mesh = mesh.translate((x_translate, z_translate, y_translate),relative=False)
        #o3d.visualization.draw_geometries([or_mesh,mesh])

        or_pcd = o3d.geometry.PointCloud(or_mesh.vertices)
        pcd = o3d.geometry.PointCloud(mesh.vertices)
        dis = pcd.compute_point_cloud_distance(or_pcd)   #por cada punto del target (argumento) busca punto mas cercano de pcd

        for v in range(len(dis)):
            values.append(dis[v]/100)
            metodos.append(methods[a])
            points.append(v)
            group.append(int((dis[v]/100)*1000)/1000)

        means.append(sum(group[-len(dis):])/len(dis))
        f.write("Objecto: "+i+" | Algoritmo: "+methods[a]+" | media: "+str(means[-1:])+"\n")
        print(" method",methods[a],"done")

    di = {'Algoritmo': metodos, 'grupo': group, 'punto': points,"valor":values} 
    df = pd.DataFrame(di)
    s = df.groupby("Algoritmo")["grupo"].value_counts(sort=False)
    order = pd.DataFrame({'Distancia':s.index.get_level_values(1), 'Cantidad_Puntos':s.values, "Algoritmo":s.index.get_level_values(0)})
    """
    fig = px.line(order, x="Distancia",y="Cantidad_Puntos",color="Algoritmo",
                    labels={
                     "Distancia": "Error (m)"
                    })
    """

    lcolor="#636EFA"
    for m in range(len(means)):
        if counter==1:
            lcolor="#EF553B"
        elif counter==2:
            lcolor="#00CC96"
        elif counter==3:
            lcolor="#AB63FA"
        elif counter==4:
            lcolor="#FFA15A"
        #fig.add_vline(x=means[m], line_width=1, line_dash="dash", line_color=color,name="Media ")
        printm = int(means[m]*1000)/1000

        maxv = order["Cantidad_Puntos"].values.max() #[(order["Algoritmo"]==methods[m]) & (order["Distancia"]==printm)]
        fig.add_trace(go.Scatter(x=s.index.get_level_values(1), 
                      y=s.values, 
                      name=i,line=dict(color=lcolor)))
        fig.add_trace(go.Scatter(x=[means[m],means[m]], 
                      y=[0,maxv], 
                      mode='lines',line=dict(color=lcolor, width=2, dash='dash'),
                      name="Media: "+str(printm)))
        print("object",i,"done")
        counter+=1

fig.update_layout(legend=dict(
    y=1,
    x=0.7,
    title=None
))

fig.update_xaxes(title_text="Error (m)")
fig.update_yaxes(title_text="Cantidad_Puntos")
fig.write_image(i+"_graph.svg")


f.close()