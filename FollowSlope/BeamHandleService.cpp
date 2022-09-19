#include "BeamHandleService.h"
#include <algorithm>

bool comp(const std::pair<double, std::vector<std::vector<TopoDS_Face>>>& a, const std::pair<double, std::vector<std::vector<TopoDS_Face>>>& b)
{
    return a.first > b.first;
}

bool comp2(const std::pair<std::vector<TopoDS_Face>, double>& a, const std::pair<std::vector<TopoDS_Face>, double>& b)
{
    return a.second > b.second;
}

std::vector<TopoDS_Edge> BeamHandleService::CreateBeamLine(std::vector<TopoDS_Face> faces)
{
    auto faceVec = ClassifyTriangle(faces);
    auto filterFaces = FilterFaceByNormal(faceVec);
    if (filterFaces.size() <= 0)
    {
        return std::vector<TopoDS_Edge>();
    }
    auto needBeamFaces = GetNeedBeamFace(filterFaces);
    return FacePoint(needBeamFaces);
}

std::vector<std::vector<TopoDS_Face>> BeamHandleService::ClassifyTriangle(std::vector<TopoDS_Face> faces)
{
    std::vector<std::pair<gp_Dir, std::vector<TopoDS_Face>>> faceGroup;
    for (TopoDS_Face face : faces)
    {
        bool has = false;
        auto dir = CalFaceDirection(face);
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

gp_Dir BeamHandleService::CalFaceDirection(TopoDS_Face face)
{
    auto edges = OCCUtils::Face::FaceEdges(face);
    auto tmpPts = OCCUtils::Edge::EdgePoints(edges[0]);
    auto pt = tmpPts[1] - tmpPts[0];
    gp_Dir dir(pt.X(), pt.Y(), pt.Z());
    gp_Dir resDir = dir;
    for (size_t i = 1; i < edges.size(); i++)
    {
        auto tmpPts = OCCUtils::Edge::EdgePoints(edges[i]);
        auto ept = tmpPts[1] - tmpPts[0];
        gp_Dir eDir(ept.X(), ept.Y(), ept.Z());
        if (!dir.IsParallel(eDir, 0.0001))
        {
            resDir = dir.Crossed(eDir);
            return resDir;
        }
    }

    return resDir;
}

std::vector<std::vector<TopoDS_Face>> BeamHandleService::FilterFaceByNormal(std::vector<std::vector<TopoDS_Face>> faceVec)
{
    std::vector<std::vector<TopoDS_Face>> resFaces;
    for (auto faces : faceVec)
    {
        if (Abs(CalFaceDirection(faces[0]).Z()) > 0.2)
        {
            resFaces.push_back(faces);
        }
    }

    return resFaces;
}

std::vector<TopoDS_Face> BeamHandleService::GetNeedBeamFace(std::vector<std::vector<TopoDS_Face>> faceVec)
{
    std::vector<std::pair<double, std::vector<std::vector<TopoDS_Face>>>> faceGroup;
    for (auto faceLst : faceVec)
    {
        double val = 0;
        for (size_t i = 0; i < faceLst.size(); i++)
        {
            val += OCCUtils::Surface::Area(faceLst[i]);
        }
        bool neweKy = true;
        for (size_t i = 0; i < faceGroup.size(); i++)
        {
            if (Abs(faceGroup[i].first - val) < 10)
            {
                faceGroup[i].second.push_back(faceLst);
                neweKy = false;
                break;
            }
        }
        if (neweKy)
        {
            std::vector<std::vector<TopoDS_Face>> tempFaces;
            tempFaces.push_back(faceLst);
            faceGroup.push_back(std::pair<double, std::vector<std::vector<TopoDS_Face>>>(val, tempFaces));
        }
    }

    sort(faceGroup.begin(), faceGroup.end(), comp);
    auto resFaceLst = faceGroup.begin()->second;

    std::vector<std::pair<std::vector<TopoDS_Face>, double>> faceDic;
    for (auto faces : resFaceLst)
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

    sort(faceDic.begin(), faceDic.end(), comp2);
    return faceDic[0].first;
}

std::vector<TopoDS_Edge> BeamHandleService::FacePoint(std::vector<TopoDS_Face> faces)
{
    std::vector<TopoDS_Edge> beamEdge;
    std::vector<gp_Pnt> facePts;
    for (auto face : faces)
    {   
        auto pts = OCCUtils::Face::FacePoints(face);
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
        }
    }

    gp_Pnt pt1 = facePts[0];
    facePts.erase(facePts.begin());
    std::vector<gp_Pnt>::iterator it;
    double distance = DBL_MAX;
    for (auto i = facePts.begin(); i != facePts.end(); ++i) {
        double dis = OCCUtils::Point::Distance(*i, pt1);
        if (dis < distance)
        {
            distance = dis;
            it = i;
        }
    }
    gp_Pnt pt2 = *it;
    facePts.erase(it);
    gp_Pnt pt3 = facePts.front();
    gp_Pnt pt4 = facePts.back();

    auto sp = pt1 + pt2;
    sp = gp_Pnt(sp.X() / 2, sp.Y() / 2, sp.Z() / 2);
    auto ep = pt3 + pt4;
    ep = gp_Pnt(ep.X() / 2, ep.Y() / 2, ep.Z() / 2);
    TopoDS_Edge beam = OCCUtils::Edge::FromPoints(sp, ep);
    beamEdge.push_back(beam);
    return beamEdge;
}