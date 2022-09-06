#include "FollowSlopeService.h"

void FollowSlopeService::FollowSlope(std::string beamPath, std::string roofPath)
{
    IGESControl_Writer ICW("MM", 0);

    IGESControl_Reader reader;
    IFSelect_ReturnStatus stat = reader.ReadFile(beamPath.c_str());
    Handle(TColStd_HSequenceOfTransient)  list = reader.GiveList();
    Standard_Integer nbtrans = reader.TransferList(list);

    IGESControl_Reader reader2;
    IFSelect_ReturnStatus stat2 = reader2.ReadFile(roofPath.c_str());
    Handle(TColStd_HSequenceOfTransient) list2 = reader2.GiveList();
    Standard_Integer nbtrans2 = reader2.TransferList(list2);

    double maxZValue = DBL_MIN;
    for (Standard_Integer i = 1; i <= reader2.NbShapes(); i++) {
        TopoDS_Shape roofShape = reader2.Shape(i);
        TopoDS_Face face = TopoDS::Face(roofShape);
        TopoDS_Wire outerWire = BRepTools::OuterWire(face);//get the outer loop
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

    maxZValue = maxZValue + 100;
    for (Standard_Integer i = 1; i <= reader.NbShapes(); i++)
    {
        TopoDS_Shape beamShape = reader.Shape(i);
        std::vector<std::pair<TopoDS_Edge, gp_Ax1>> edgeMap;
        for (Standard_Integer i = 1; i <= reader2.NbShapes(); i++)
        {
            TopoDS_Shape roofShape = reader2.Shape(i);
            TopoDS_Face face = TopoDS::Face(roofShape);

            Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
            GeomLib_IsPlanarSurface isPlanar(surface);
            if (isPlanar.IsPlanar()) {
                Handle(Geom_Plane) plane = new Geom_Plane(isPlanar.Plan());
                gp_Ax1 dir = plane->Axis();
                std::vector<gp_Pnt> vertices;
                for (TopExp_Explorer expVert(TopoDS::Edge(beamShape), TopAbs_VERTEX); expVert.More(); expVert.Next())
                {
                    TopoDS_Vertex vert = TopoDS::Vertex(expVert.Current());
                    vertices.push_back(BRep_Tool::Pnt(vert));
                }
                gp_Pnt pt1 = vertices[0];
                gp_Pnt pt2 = vertices[1];
                auto pt3 = gp_Pnt(pt2.X(), pt2.Y(), pt2.Z() + maxZValue);
                auto pt4 = gp_Pnt(pt1.X(), pt1.Y(), pt1.Z() + maxZValue);
                BRepBuilderAPI_MakePolygon maker(pt1, pt2, pt3, pt4, Standard_True);
                BRepBuilderAPI_MakeFace faceMaker(maker.Wire());
                BRepAlgoAPI_Section section(face, faceMaker.Face());
                for (TopExp_Explorer explorer(section.Shape(), TopAbs_EDGE); explorer.More(); explorer.Next())
                {
                    TopoDS_Shape shape = explorer.Value();
                    TopoDS_Edge edge = TopoDS::Edge(shape);
                    edgeMap.push_back(std::pair<TopoDS_Edge, gp_Ax1>(edge, dir));
                }
            }
        }

        if (edgeMap.size() > 0)
        {
            auto resEdges = CreateCompleteBeam(edgeMap);
            auto beamEdges = CreateBeamSolidEdges(resEdges);
            for (size_t i = 0; i < beamEdges.size(); i++)
            {
                ICW.AddShape(beamEdges[i]);
            }
        }
    }

    ICW.ComputeModel();
    ICW.Write("D:\\result.igs");
}

std::vector<TopoDS_Edge> FollowSlopeService::CreateBeamSolidEdges(std::vector<std::pair<TopoDS_Edge, gp_Ax1>> edges)
{
    std::vector<TopoDS_Edge> resEdges;
    for (auto edgeIt : edges)
    {
        auto edge = edgeIt.first;
        auto dir = edgeIt.second.Direction();

        std::vector<gp_Pnt> vertices;
        for (TopExp_Explorer expVert(edge, TopAbs_VERTEX); expVert.More(); expVert.Next())
        {
            TopoDS_Vertex vert = TopoDS::Vertex(expVert.Current());
            vertices.push_back(BRep_Tool::Pnt(vert));
        }
        gp_Pnt pt1 = vertices[0];
        gp_Pnt pt2 = vertices[1];
        auto fPt = pt2 - pt1;
        gp_Dir zDir(0, 0, 1);
        gp_Dir xDir(fPt.X(), fPt.Y(), fPt.Z());
        gp_Dir yDir = xDir.Crossed(zDir);
        if (!zDir.IsParallel(dir, 0.001))
        {
            gp_Dir checkDir = dir.Crossed(dir.Crossed(zDir));
            if (!xDir.IsParallel(checkDir, 0.001))
            {
                yDir = checkDir;
            }
        }
        
        auto p1 = pt1 + yDir * 5;
        auto p2 = pt2 + yDir * 5;
        auto p3 = pt2 - yDir * 5;
        auto p4 = pt1 - yDir * 5;

        auto p5 = p1 - zDir * 5;
        auto p6 = p2 - zDir * 5;
        auto p7 = p3 - zDir * 5;;
        auto p8 = p4 - zDir * 5;

        resEdges.push_back(OCCUtils::Edge::FromPoints(p1, p2));
        resEdges.push_back(OCCUtils::Edge::FromPoints(p2, p3));
        resEdges.push_back(OCCUtils::Edge::FromPoints(p3, p4));
        resEdges.push_back(OCCUtils::Edge::FromPoints(p4, p1));
        resEdges.push_back(OCCUtils::Edge::FromPoints(p5, p6));
        resEdges.push_back(OCCUtils::Edge::FromPoints(p6, p7));
        resEdges.push_back(OCCUtils::Edge::FromPoints(p7, p8));
        resEdges.push_back(OCCUtils::Edge::FromPoints(p8, p5));
        resEdges.push_back(OCCUtils::Edge::FromPoints(p1, p5));
        resEdges.push_back(OCCUtils::Edge::FromPoints(p2, p6));
        resEdges.push_back(OCCUtils::Edge::FromPoints(p3, p7));
        resEdges.push_back(OCCUtils::Edge::FromPoints(p4, p8));
    }

    return resEdges;
}

/// <summary>
/// 将碎梁线合并成整梁线
/// </summary>
/// <param name="edges"></param>
/// <returns></returns>
std::vector<std::pair<TopoDS_Edge, gp_Ax1>>  FollowSlopeService::CreateCompleteBeam(std::vector<std::pair<TopoDS_Edge, gp_Ax1>> edges)
{
    std::vector<std::pair<std::vector<TopoDS_Edge>, gp_Ax1>> groupEdges;
    std::vector<TopoDS_Edge> completeEdges;
    auto dir = edges.begin()->second;
    while (edges.size() > 0)
    {
        if (completeEdges.size() == 0)
        {
            auto firEdgeLst = edges.begin()->first;
            dir = edges.begin()->second;
            edges.erase(edges.begin());
            completeEdges.push_back(firEdgeLst);
        }

        gp_Pnt fSp, fEp;
        auto completeEdge = MergeBeamLines(completeEdges, fSp, fEp);
        auto fPt = fEp - fSp;
        gp_Dir fDir(fPt.X(), fPt.Y(), fPt.Z());
        bool change = false;
        auto it = edges.begin();
        while (it != edges.end())
        {
            std::vector<gp_Pnt> vertices;
            for (TopExp_Explorer expVert(it->first, TopAbs_VERTEX); expVert.More(); expVert.Next())
            {
                TopoDS_Vertex vert = TopoDS::Vertex(expVert.Current());
                vertices.push_back(BRep_Tool::Pnt(vert));
            }
            gp_Pnt pt1 = vertices[0];
            gp_Pnt pt2 = vertices[1];
            auto ePt = pt1 - pt2;
            gp_Dir eDir(ePt.X(), ePt.Y(), ePt.Z());
            if (((abs(pt1.X() - fSp.X()) < 0.01 && abs(pt1.Y() - fSp.Y()) < 0.01 && abs(pt1.Z() - fSp.Z()) < 0.01) ||
                (abs(pt1.X() - fEp.X()) < 0.01 && abs(pt1.Y() - fEp.Y()) < 0.01 && abs(pt1.Z() - fEp.Z()) < 0.01) ||
                (abs(pt2.X() - fSp.X()) < 0.01 && abs(pt2.Y() - fSp.Y()) < 0.01 && abs(pt2.Z() - fSp.Z()) < 0.01) ||
                (abs(pt2.X() - fEp.X()) < 0.01 && abs(pt2.Y() - fEp.Y()) < 0.01 && abs(pt2.Z() - fEp.Z()) < 0.01)) &&
                fDir.IsParallel(eDir, 0.0001))
            {
                completeEdges.push_back(it->first);
                change = true;
                it = edges.erase(it);
            }
            else
            {
                it++;
            }
        }
        if (!change)
        {
            groupEdges.push_back(std::pair<std::vector<TopoDS_Edge>, gp_Ax1>(completeEdges, dir));
            completeEdges.clear();
        }
    }
    if (completeEdges.size() > 0)
    {
        groupEdges.push_back(std::pair<std::vector<TopoDS_Edge>, gp_Ax1>(completeEdges, dir));
    }
    std::vector<std::pair<TopoDS_Edge, gp_Ax1>>  resEdges;
    for (auto it : groupEdges)
    {
        gp_Pnt tempPt1, tempPt2;
        resEdges.push_back(std::pair<TopoDS_Edge, gp_Ax1>(MergeBeamLines(it.first, tempPt1, tempPt2), it.second));
    }

    return resEdges;
}

/// <summary>
/// 合并beamline为一条线
/// </summary>
/// <param name="edges"></param>
/// <returns></returns>
TopoDS_Edge FollowSlopeService::MergeBeamLines(std::vector<TopoDS_Edge> edges, gp_Pnt& pt1, gp_Pnt& pt2)
{
    std::vector<gp_Pnt> ptLst;
    for (size_t i = 0; i < edges.size(); i++)
    {
        for (TopExp_Explorer expVert(edges[0], TopAbs_VERTEX); expVert.More(); expVert.Next())
        {
            TopoDS_Vertex vert = TopoDS::Vertex(expVert.Current());
            ptLst.push_back(BRep_Tool::Pnt(vert));
        }
    }
    auto firPt = ptLst[0];
    gp_Pnt sp = firPt;
    double dis = OCCUtils::Point::Distance(firPt, sp);
    for (size_t i = 0; i < ptLst.size(); i++)
    {
        gp_Pnt pt = ptLst[i];
        double ptDis = OCCUtils::Point::Distance(firPt, pt);
        if (ptDis > dis)
        {
            dis = ptDis;
            sp = pt;
        }
    }

    gp_Pnt ep = sp;
    double endDis = OCCUtils::Point::Distance(ep, sp);
    for (size_t i = 0; i < ptLst.size(); i++)
    {
        gp_Pnt pt = ptLst[i];
        double ptDis = OCCUtils::Point::Distance(sp, pt);
        if (ptDis > endDis)
        {
            endDis = ptDis;
            ep = pt;
        }
    }

    auto beamEdge = BRepBuilderAPI_MakeEdge(sp, ep).Edge();
    pt1 = sp;
    pt2 = ep;
    return beamEdge;
}