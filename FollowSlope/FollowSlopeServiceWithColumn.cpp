#include "FollowSlopeServiceWithColumn.h"
#include "BeamHandleService.h"
#include "ColumnHandleService.h"
#include "FollowSlopeService.h"
#include <io.h>  
#include <iostream>

FollowSlopeService service;
void FollowSlopeServiceWithColumn::FollowSlope(std::string beamPath, std::string roofPath, std::string columnPath)
{
    IGESControl_Writer BICW("BEAM", 0);
    IGESControl_Writer CICW("COL", 0);

    std::vector<std::string> beamPathList;
    getFiles(beamPath, beamPathList);
    std::vector<TopoDS_Edge> beamEdges;
    for (size_t i = 0; i < beamPathList.size(); i++)
    {
        IGESControl_Reader beamReader;
        IFSelect_ReturnStatus stat = beamReader.ReadFile(beamPathList[i].c_str());
        Handle(TColStd_HSequenceOfTransient)  list = beamReader.GiveList();
        Standard_Integer nbtrans = beamReader.TransferList(list);
        auto edgeLst = GetBeamEdge(beamReader);
        if (edgeLst.size() > 0)
        {
            auto tempEdge = edgeLst[0];
            beamEdges.push_back(tempEdge);
        }
    }

    std::vector<std::string> columnPathList;
    getFiles(columnPath, columnPathList);
    std::vector<std::vector<TopoDS_Edge>> columnEdges;
    for (size_t i = 0; i < columnPathList.size(); i++)
    {
        IGESControl_Reader columnReader;
        IFSelect_ReturnStatus stat = columnReader.ReadFile(columnPathList[i].c_str());
        Handle(TColStd_HSequenceOfTransient)  list = columnReader.GiveList();
        Standard_Integer nbtrans = columnReader.TransferList(list);
        auto edgeLst = GetColumnEdge(columnReader);
        if (edgeLst.size() > 0)
        {
            auto tempEdge = edgeLst;
            columnEdges.push_back(tempEdge);
        }
    }

    std::vector<std::string> roofPathList;
    getFiles(roofPath, roofPathList);
    IGESControl_Reader roofReader;
    IFSelect_ReturnStatus stat2 = roofReader.ReadFile(roofPathList[0].c_str());
    Handle(TColStd_HSequenceOfTransient) list2 = roofReader.GiveList();
    Standard_Integer nbtrans2 = roofReader.TransferList(list2);

    double maxZValue = DBL_MIN;
    for (Standard_Integer i = 1; i <= roofReader.NbShapes(); i++) {
        TopoDS_Shape roofShape = roofReader.Shape(i);
        auto type = roofShape.ShapeType();
        if (type == TopAbs_ShapeEnum::TopAbs_FACE)
        {
            TopoDS_Face face = TopoDS::Face(roofShape);
            for (TopExp_Explorer expVert(face, TopAbs_VERTEX); expVert.More(); expVert.Next())
            {
                TopoDS_Vertex roofVert = TopoDS::Vertex(expVert.Current());
                auto roofPt = BRep_Tool::Pnt(roofVert);
                if (maxZValue < roofPt.Z())
                {
                    maxZValue = roofPt.Z();
                }
            }
        }
    }

    maxZValue = maxZValue + 100;
    auto beamSolids = BeamFollowSlope(beamEdges, maxZValue, roofReader);
    for (size_t i = 0; i < beamSolids.size(); i++)
    {
        BICW.AddShape(beamSolids[i]);
    }
    BICW.ComputeModel();
    BICW.Write("D:\\BeamResult.igs");

    auto columnsFaces = ColumnFollowSlope(columnEdges, maxZValue, roofReader);
    for (size_t i = 0; i < columnsFaces.size(); i++)
    {
        CICW.AddShape(columnsFaces[i]);
    }
    CICW.ComputeModel();
    CICW.Write("D:\\ColumnResult.igs");
}

/// <summary>
/// 获取梁中线
/// </summary>
/// <param name="reder"></param>
/// <returns></returns>
std::vector<TopoDS_Edge> FollowSlopeServiceWithColumn::GetBeamEdge(IGESControl_Reader reder)
{
    std::vector<TopoDS_Face> beamFaces;
    for (Standard_Integer i = 1; i <= reder.NbShapes(); i++)
    {
        TopoDS_Shape beamShape = reder.Shape(i);
        auto face = TopoDS::Face(beamShape);
        beamFaces.push_back(face);
    }
    BeamHandleService service;
    return service.CreateBeamLine(beamFaces);
}

/// <summary>
/// 获取柱边线
/// </summary>
/// <param name=""></param>
/// <returns></returns>
std::vector<TopoDS_Edge> FollowSlopeServiceWithColumn::GetColumnEdge(IGESControl_Reader reder)
{
    std::vector<TopoDS_Face> beamFaces;
    for (Standard_Integer i = 1; i <= reder.NbShapes(); i++)
    {
        TopoDS_Shape beamShape = reder.Shape(i);
        auto face = TopoDS::Face(beamShape);
        beamFaces.push_back(face);
    }
    ColumnHandleService service;
    return service.CreateColumnLine(beamFaces);
}

/// <summary>
/// 梁随坡
/// </summary>
/// <param name="beamEdges"></param>
/// <param name="maxZValue"></param>
/// <param name="roofReader"></param>
/// <returns></returns>
std::vector<TopoDS_Face> FollowSlopeServiceWithColumn::BeamFollowSlope(std::vector<TopoDS_Edge> beamEdges, double maxZValue, IGESControl_Reader roofReader)
{
    std::vector<TopoDS_Face> beamSolids;
    for (Standard_Integer i = 0; i < beamEdges.size(); i++)
    {
        TopoDS_Edge beamShape = beamEdges[i];
        std::vector<std::pair<TopoDS_Edge, gp_Dir>> edgeMap;

        std::vector<gp_Pnt> vertices = OCCUtils::Edge::EdgePoints(beamShape);
        gp_Pnt pt1 = vertices[0];
        gp_Pnt pt2 = vertices[1];
        auto pt3 = gp_Pnt(pt2.X(), pt2.Y(), pt2.Z() + maxZValue);
        auto pt4 = gp_Pnt(pt1.X(), pt1.Y(), pt1.Z() + maxZValue);
        BRepBuilderAPI_MakePolygon maker(pt1, pt2, pt3, pt4, Standard_True);
        BRepBuilderAPI_MakeFace faceMaker(maker.Wire());
        for (Standard_Integer i = 1; i <= roofReader.NbShapes(); i++)
        {
            TopoDS_Shape roofShape = roofReader.Shape(i);
            if (roofShape.ShapeType() == TopAbs_ShapeEnum::TopAbs_FACE) {
                TopoDS_Face face = TopoDS::Face(roofShape);
                Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
                GeomLib_IsPlanarSurface isPlanar(surface);
                BRepAlgoAPI_Section section(face, faceMaker.Face());
                gp_Dir dir;
                bool first = true;
                for (TopExp_Explorer explorer(section.Shape(), TopAbs_EDGE); explorer.More(); explorer.Next())
                {
                    TopoDS_Shape shape = explorer.Value();
                    TopoDS_Edge edge = TopoDS::Edge(shape);
                    if (OCCUtils::Edge::Length(edge) > 1)
                    {
                        if (first)
                        {
                            if (!service.CalFaceDirection(face, edge, dir)) {
                                continue;
                            }
                            first = false;
                        }
                        edgeMap.push_back(std::pair<TopoDS_Edge, gp_Dir>(edge, dir));
                    }
                }
            }
        }

        if (edgeMap.size() > 0)
        {
            auto resEdges = service.CreateCompleteBeam(edgeMap);
            auto tempBeamSolids = service.CreateBeamSolidSolids(resEdges);
            for (size_t i = 0; i < tempBeamSolids.size(); i++)
            {
                beamSolids.push_back(tempBeamSolids[i]);
            }
        }
    }
    return beamSolids;
}

/// <summary>
/// 柱随坡
/// </summary>
/// <param name="columnEdges"></param>
/// <param name="maxZValue"></param>
/// <param name="roofReader"></param>
/// <returns></returns>
std::vector<TopoDS_Face> FollowSlopeServiceWithColumn::ColumnFollowSlope(std::vector <std::vector<TopoDS_Edge>> columnEdges, double maxZValue, IGESControl_Reader roofReader)
{
    std::vector<TopoDS_Face> columnFaces;
    for (Standard_Integer i = 0; i < columnEdges.size(); i++)
    {
        std::vector<TopoDS_Edge> sectionEdges;
        for (Standard_Integer j = 0; j < columnEdges[i].size(); j++)
        {
            TopoDS_Edge beamShape = columnEdges[i][j];
            std::vector<TopoDS_Edge> edgeVec;
            std::vector<gp_Pnt> vertices = OCCUtils::Edge::EdgePoints(beamShape);
            gp_Pnt pt1 = vertices[0];
            gp_Pnt pt2 = vertices[1];
            auto pt3 = gp_Pnt(pt2.X(), pt2.Y(), pt2.Z() + maxZValue);
            auto pt4 = gp_Pnt(pt1.X(), pt1.Y(), pt1.Z() + maxZValue);
            BRepBuilderAPI_MakePolygon maker(pt1, pt2, pt3, pt4, Standard_True);
            BRepBuilderAPI_MakeFace faceMaker(maker.Wire());
            for (Standard_Integer i = 1; i <= roofReader.NbShapes(); i++)
            {
                TopoDS_Shape roofShape = roofReader.Shape(i);
                if (roofShape.ShapeType() == TopAbs_ShapeEnum::TopAbs_FACE) {
                    TopoDS_Face face = TopoDS::Face(roofShape);
                    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
                    GeomLib_IsPlanarSurface isPlanar(surface);
                    BRepAlgoAPI_Section section(face, faceMaker.Face());
                    for (TopExp_Explorer explorer(section.Shape(), TopAbs_EDGE); explorer.More(); explorer.Next())
                    {
                        TopoDS_Shape shape = explorer.Value();
                        TopoDS_Edge edge = TopoDS::Edge(shape);
                        edgeVec.push_back(edge);
                        sectionEdges.push_back(edge);
                    }
                }
            }
            if (edgeVec.size() > 0)
            {
                columnFaces.push_back(CreateColumnSolidFaces(edgeVec, beamShape));
            }
        }
        //columnFaces.push_back(CreateColumnTopSolidFaces(sectionEdges));
    }

    return columnFaces;
}

/// <summary>
/// 创建完整柱
/// </summary>
/// <param name="edges"></param>
/// <param name="edge"></param>
/// <returns></returns>
TopoDS_Face FollowSlopeServiceWithColumn::CreateColumnSolidFaces(std::vector<TopoDS_Edge> edges, TopoDS_Edge edge)
{
    auto pts = OCCUtils::Edge::EdgePoints(edge);
    gp_Dir dir(0, 0, -1);
    BRepBuilderAPI_MakePolygon maker1;
    maker1.Add(pts[0]);
    maker1.Add(pts[1]);
    gp_Pnt spt = pts[1];

    while (edges.size() > 0)
    {
        double distance = DBL_MAX;
        bool start = true;
        std::vector<TopoDS_Edge>::iterator it;
        for (auto i = edges.begin(); i != edges.end(); ++i) {
            auto tempPts = OCCUtils::Edge::EdgePoints(*i);
            double sDis = OCCUtils::Point::Distance(spt, tempPts[0]);
            if (sDis < distance)
            {
                distance = sDis;
                start = true;
                it = i;
            }

            double eDis = OCCUtils::Point::Distance(spt, tempPts[1]);
            if (eDis < distance)
            {
                distance = eDis;
                start = false;
                it = i;
            }
        }

        auto nextPts = OCCUtils::Edge::EdgePoints(*it);
        if (start)
        {
            maker1.Add(nextPts[0]);
            maker1.Add(nextPts[1]);
            spt = nextPts[1];
        }
        else
        {
            maker1.Add(nextPts[1]);
            maker1.Add(nextPts[0]);
            spt = nextPts[0];
        }
        edges.erase(it);
    }
    maker1.Add(pts[0] + dir * 400);

    auto face1 = BRepBuilderAPI_MakeFace(maker1.Wire()).Face();
    return face1;
}

/// <summary>
/// 创建柱/墙顶部面
/// </summary>
/// <param name="edges"></param>
/// <returns></returns>
TopoDS_Face FollowSlopeServiceWithColumn::CreateColumnTopSolidFaces(std::vector<TopoDS_Edge> edges)
{
    auto pts = OCCUtils::Edge::EdgePoints(*edges.begin());
    edges.erase(edges.begin());
    BRepBuilderAPI_MakePolygon maker1;
    maker1.Add(pts[0]);
    maker1.Add(pts[1]);
    gp_Pnt spt = pts[1];
    while (edges.size() > 0)
    {
        double distance = DBL_MAX;
        bool start = true;
        std::vector<TopoDS_Edge>::iterator it;
        for (auto i = edges.begin(); i != edges.end(); ++i) {
            auto tempPts = OCCUtils::Edge::EdgePoints(*i);
            double sDis = OCCUtils::Point::Distance(spt, tempPts[0]);
            if (sDis < distance)
            {
                distance = sDis;
                start = true;
                it = i;
            }

            double eDis = OCCUtils::Point::Distance(spt, tempPts[1]);
            if (eDis < distance)
            {
                distance = eDis;
                start = false;
                it = i;
            }
        }

        auto nextPts = OCCUtils::Edge::EdgePoints(*it);
        if (start)
        {
            maker1.Add(nextPts[0]);
            maker1.Add(nextPts[1]);
            spt = nextPts[1];
        }
        else
        {
            maker1.Add(nextPts[1]);
            maker1.Add(nextPts[0]);
            spt = nextPts[0];
        }
        edges.erase(it);
    }
    maker1.Add(pts[0]);

    auto face1 = BRepBuilderAPI_MakeFace(maker1.Wire()).Face();
    return face1;
}

/// <summary>
/// 获取所有文件名
/// </summary>
/// <param name="path"></param>
/// <param name="files"></param>
void FollowSlopeServiceWithColumn::getFiles(std::string path, std::vector<std::string>& files)
{
    //文件句柄  
    intptr_t hFile = 0;
    //文件信息，声明一个存储文件信息的结构体  
    struct _finddata_t fileinfo;
    std::string p;  //字符串，存放路径
    if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)//若查找成功，则进入
    {
        do
        {
            //如果是目录,迭代之（即文件夹内还有文件夹）  
            if ((fileinfo.attrib & _A_SUBDIR))
            {
                //文件名不等于"."&&文件名不等于".."
                //.表示当前目录
                //..表示当前目录的父目录
                //判断时，两者都要忽略，不然就无限递归跳不出去了！
                if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
                    getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
            }
            //如果不是,加入列表  
            else
            {
                files.push_back(p.assign(path).append("\\").append(fileinfo.name));
            }
        } while (_findnext(hFile, &fileinfo) == 0);
        //_findclose函数结束查找
        _findclose(hFile);
    }
}

/// <summary>
/// 创建完整梁线
/// </summary>
/// <param name="edgeVec"></param>
/// <param name="pt1"></param>
/// <param name="pt2"></param>
/// <returns></returns>
std::vector<std::pair<TopoDS_Edge, gp_Dir>> FollowSlopeServiceWithColumn::CreateCompleteBeam(std::vector<std::pair<TopoDS_Edge, gp_Dir>> edgeVec, gp_Pnt pt1, gp_Pnt pt2)
{
    std::vector<gp_Pnt> pts;
    gp_Pnt spt = pt1;
    while (edgeVec.size() > 0)
    {
        double distance = DBL_MAX;
        bool start = true;
        std::vector<std::pair<TopoDS_Edge, gp_Dir>>::iterator it;
        for (auto i = edgeVec.begin(); i != edgeVec.end(); ++i) {
            auto tempPts = OCCUtils::Edge::EdgePoints(i->first);
            double sDis = abs(spt.X() - tempPts[0].X()) + abs(spt.Y() - tempPts[0].Y());
            if (sDis < distance)
            {
                distance = sDis;
                start = true;
                it = i;
            }

            double eDis = OCCUtils::Point::Distance(spt, tempPts[1]);
            if (eDis < distance)
            {
                distance = eDis;
                start = false;
                it = i;
            }
        }

        auto nextPts = OCCUtils::Edge::EdgePoints(*it);
        if (start)
        {
            maker1.Add(nextPts[0]);
            maker1.Add(nextPts[1]);
            spt = nextPts[1];
        }
        else
        {
            maker1.Add(nextPts[1]);
            maker1.Add(nextPts[0]);
            spt = nextPts[0];
        }
        edges.erase(it);
    }
    maker1.Add(pts[0]);

    auto face1 = BRepBuilderAPI_MakeFace(maker1.Wire()).Face();
    return std::vector<std::pair<TopoDS_Edge, gp_Dir>>();
}
