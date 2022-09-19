#include "ColumnHandleService.h"
#include "BeamHandleService.h"
#include <algorithm>

bool comp(const std::pair<std::vector<TopoDS_Face>, double>& a, const std::pair<std::vector<TopoDS_Face>, double>& b)
{
    return a.second > b.second;
}

std::vector<TopoDS_Edge> ColumnHandleService::CreateColumnLine(std::vector<TopoDS_Face> faces)
{
    BeamHandleService service;
    auto faceVec = ClassifyTriangle(faces);
    auto filterFaces = service.FilterFaceByNormal(faceVec);
    auto needBeamFaces = GetNeedColumnFace(filterFaces);
    return FaceEdges(needBeamFaces);
}

/// <summary>
/// 分类三角面片
/// </summary>
/// <param name="faces"></param>
/// <returns></returns>
std::vector<std::vector<TopoDS_Face>> ColumnHandleService::ClassifyTriangle(std::vector<TopoDS_Face> faces)
{
    BeamHandleService service;
    std::vector<std::pair<gp_Dir, std::vector<TopoDS_Face>>> faceGroup;
    for (TopoDS_Face face : faces)
    {
        bool has = false;
        auto dir = service.CalFaceDirection(face);
        for (size_t i = 0; i < faceGroup.size(); i++)
        {
            if (faceGroup[i].first.IsParallel(dir, 0.001))
            {
                faceGroup[i].second.push_back(face);
                has = true;
                break;
            }
        }
        if (!has)
        {
            std::vector<TopoDS_Face> valueFaces;
            valueFaces.push_back(face);
            faceGroup.push_back(std::pair<gp_Dir, std::vector<TopoDS_Face>>(dir, valueFaces));
        }
    }

    std::vector<std::vector<TopoDS_Face>> lineStrGroup;
    for (auto group : faceGroup)
    {
        std::vector<TopoDS_Face> groupLineStrings = group.second;
        std::vector<TopoDS_Face> faceVec;
        while (groupLineStrings.size() > 0)
        {
            if (faceVec.size() <= 0)
            {
                auto firLinestring = groupLineStrings[0];
                groupLineStrings.erase(groupLineStrings.begin());
                faceVec.push_back(firLinestring);
            }

            std::vector<TopoDS_Face> closeFaces;
            for (size_t i = 0; i < groupLineStrings.size(); i++)
            {
                for (size_t j = 0; j < faceVec.size(); j++)
                {
                    if (OCCUtils::Face::FaceIntersect(groupLineStrings[i], faceVec[j]))
                    {
                        closeFaces.push_back(groupLineStrings[i]);
                        break;
                    }
                }
            }
            if (closeFaces.size() <= 0)
            {
                lineStrGroup.push_back(faceVec);
                faceVec.clear();
                continue;
            }

            auto it = groupLineStrings.begin();
            while (it != groupLineStrings.end())
            {
                bool isErase = false;
                for (auto face : closeFaces)
                {
                    if (*it == face)
                    {
                        it = groupLineStrings.erase(it);
                        isErase = true;
                        break;
                    }
                }
                if (!isErase)
                {
                    it++;
                }
            }
            for (auto face : closeFaces)
            {
                faceVec.push_back(face);
            }
        }
        if (faceVec.size() > 0)
        {
            lineStrGroup.push_back(faceVec);
        }
    }

    return lineStrGroup;
}

/// <summary>
/// 计算需要的柱面
/// </summary>
/// <param name="faceVec"></param>
/// <returns></returns>
std::vector<TopoDS_Face> ColumnHandleService::GetNeedColumnFace(std::vector<std::vector<TopoDS_Face>> faceVec)
{
    std::vector<std::pair<std::vector<TopoDS_Face>, double>> faceDic;
    for (auto faces : faceVec)
    {
        double z = DBL_MIN;
        for (auto face : faces)
        {
            auto pts = OCCUtils::Face::FacePoints(face);
            for (size_t i = 0; i < pts.size(); i++)
            {
                if (z < pts[i].Z())
                {
                    z = pts[i].Z();
                }
            }
        }
        faceDic.push_back(std::pair<std::vector<TopoDS_Face>, double>(faces, z));
    }

    sort(faceDic.begin(), faceDic.end(), comp);
    return faceDic[0].first;
}

/// <summary>
/// 计算柱的四条面线
/// </summary>
/// <param name="faces"></param>
/// <returns></returns>
std::vector<TopoDS_Edge> ColumnHandleService::FaceEdges(std::vector<TopoDS_Face> faces)
{
    std::vector<TopoDS_Edge> columnEdges;
    std::vector<gp_Pnt> facePts;
    for (auto face : faces)
    {
        auto edges = OCCUtils::Face::FaceEdges(face);
        for (size_t i = 0; i < edges.size(); i++)
        {
            columnEdges.push_back(edges[i]);
        }
        /*auto pts = OCCUtils::Face::FacePoints(face);
        for (size_t i = 0; i < pts.size(); i++)
        {
            bool need = true;
            for (size_t j = 0; j < facePts.size(); j++)
            {
                auto pt = facePts[j];
                if (abs(pts[i].X() - pt.X()) < 0.01 && abs(pts[i].Y() - pt.Y()) < 0.01 && abs(pts[i].Z() - pt.Z()) < 0.01)
                {
                    need = false;
                }
            }
            if (need)
            {
                facePts.push_back(pts[i]);
            }
        }*/
    }
  /*  gp_Pnt pt1 = facePts[0];
    facePts.erase(facePts.begin());
    std::vector<gp_Pnt>::iterator it;
    double distance = DBL_MAX;
    for (auto i = facePts.begin(); i != facePts.end(); ++i) {
        if (OCCUtils::Point::Distance(*i, pt1) < distance)
        {
            it = i;
        }
    }
    gp_Pnt pt2 = *it;
    facePts.erase(it);
    distance = DBL_MAX;
    for (auto i = facePts.begin(); i != facePts.end(); ++i) {
        if (OCCUtils::Point::Distance(*i, pt2) < distance)
        {
            it = i;
        }
    }
    gp_Pnt pt3 = *it;
    facePts.erase(it);
    gp_Pnt pt4 = facePts.back();


    TopoDS_Edge edge1 = OCCUtils::Edge::FromPoints(pt1, pt2);
    columnEdges.push_back(edge1);
    TopoDS_Edge edge2 = OCCUtils::Edge::FromPoints(pt2, pt3);
    columnEdges.push_back(edge2);
    TopoDS_Edge edge3 = OCCUtils::Edge::FromPoints(pt3, pt4);
    columnEdges.push_back(edge3);
    TopoDS_Edge edge4 = OCCUtils::Edge::FromPoints(pt4, pt1);
    columnEdges.push_back(edge4);*/

    return columnEdges;
}
