/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#include "Map.h"
#include "KeyFrame.h"
#include "MapPoint.h"

#include <boost/uuid/nil_generator.hpp>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid.hpp>
#include <mutex>

namespace ORB_SLAM3 {

long unsigned int Map::nNextId = 0;

Map::Map()
  : mnMaxKFid(0)
  , creatorAgentId(0)
  , mnBigChangeIdx(0)
  , mbImuInitialized(false)
  , mnMapChange(0)
  , mpFirstRegionKF(static_cast<KeyFrame*>(NULL))
  , mbFail(false)
  , mIsInUse(false)
  , mHasTumbnail(false)
  , mbBad(false)
  , mnMapChangeNotified(0)
  , mbIsInertial(false)
  , mbIMU_BA1(false)
  , mbIMU_BA2(false) {
  mnId = nNextId++;
  uuid = boost::uuids::random_generator()();
  mThumbnail = static_cast<GLubyte*>(NULL);
}

Map::Map(int initKFid, unsigned int creatorAgentId)
  : mnInitKFid(initKFid)
  , mnMaxKFid(initKFid)
  , creatorAgentId(creatorAgentId)
  ,
  /*mnLastLoopKFid(initKFid),*/ mnBigChangeIdx(0)
  , mIsInUse(false)
  , mHasTumbnail(false)
  , mbBad(false)
  , mbImuInitialized(false)
  , mpFirstRegionKF(static_cast<KeyFrame*>(NULL))
  , mnMapChange(0)
  , mbFail(false)
  , mnMapChangeNotified(0)
  , mbIsInertial(false)
  , mbIMU_BA1(false)
  , mbIMU_BA2(false) {
  mnId = nNextId++;
  uuid = boost::uuids::random_generator()();
  mThumbnail = static_cast<GLubyte*>(NULL);
}

Map::~Map() {
  // TODO: erase all points from memory
  mspMapPoints.clear();

  // TODO: erase all keyframes from memory
  mspKeyFrames.clear();

  if (mThumbnail)
    delete mThumbnail;
  mThumbnail = static_cast<GLubyte*>(NULL);

  mvpReferenceMapPoints.clear();
  mvpKeyFrameOrigins.clear();
}

void Map::AddKeyFrame(KeyFrame* pKF) {
  unique_lock<mutex> lock(mMutexMap);
  if (mspKeyFrames.empty()) {
    cout << "First KF:" << pKF->mnId << "; Map init KF:" << mnInitKFid << endl;
    mnInitKFid = pKF->mnId;
    mpKFinitial = pKF;
    mpKFlowerID = pKF;
  }
  mspKeyFrames.insert(pKF);
  if (pKF->mnId > mnMaxKFid) {
    mnMaxKFid = pKF->mnId;
  }
  if (pKF->mnId < mpKFlowerID->mnId) {
    mpKFlowerID = pKF;
  }
}

void Map::AddMapPoint(MapPoint* pMP) {
  unique_lock<mutex> lock(mMutexMap);
  mspMapPoints.insert(pMP);
}

void Map::SetImuInitialized() {
  unique_lock<mutex> lock(mMutexMap);
  mbImuInitialized = true;
}

bool Map::isImuInitialized() {
  unique_lock<mutex> lock(mMutexMap);
  return mbImuInitialized;
}

void Map::EraseMapPoint(MapPoint* pMP) {
  // unique_lock<mutex> lock(mMutexMap);
  mspMapPoints.erase(pMP);

  // TODO: This only erase the pointer.
  // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame* pKF) {
  unique_lock<mutex> lock(mMutexMap);
  mspKeyFrames.erase(pKF);
  if (mspKeyFrames.size() > 0) {
    if (pKF->mnId == mpKFlowerID->mnId) {
      vector<KeyFrame*> vpKFs = vector<KeyFrame*>(mspKeyFrames.begin(), mspKeyFrames.end());
      sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);
      mpKFlowerID = vpKFs[0];
    }
    if (pKF->mnId == mnMaxKFid) {
      vector<KeyFrame*> vpKFs = vector<KeyFrame*>(mspKeyFrames.begin(), mspKeyFrames.end());
      sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);
      reverse(vpKFs.begin(), vpKFs.end());
      mnMaxKFid = vpKFs[0]->mnId;
    }
    if (pKF == mpKFinitial) {
      // Arbritarily set inital KF as the KF with the lowest id??
      // TODO: think about if this is ok and improve this code
      vector<KeyFrame*> vpKFs = vector<KeyFrame*>(mspKeyFrames.begin(), mspKeyFrames.end());
      sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);
      mpKFinitial = vpKFs[0];
      mnInitKFid = vpKFs[0]->mnId;
    }
    // what even is mvpKeyFrameOrigins used for? who populates it??
    // Is this necessary??
    if (find(mvpKeyFrameOrigins.begin(), mvpKeyFrameOrigins.end(), pKF) != mvpKeyFrameOrigins.end()) {
      mvpKeyFrameOrigins.erase(
        remove(mvpKeyFrameOrigins.begin(), mvpKeyFrameOrigins.end(), pKF), mvpKeyFrameOrigins.end());
      // TODO: replace with something?
    }
  }
  else {
    mpKFlowerID = 0;
  }

  // TODO: This only erase the pointer.
  // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint*>& vpMPs) {
  unique_lock<mutex> lock(mMutexMap);
  mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange() {
  unique_lock<mutex> lock(mMutexMap);
  mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx() {
  unique_lock<mutex> lock(mMutexMap);
  return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames() {
  unique_lock<mutex> lock(mMutexMap);
  return vector<KeyFrame*>(mspKeyFrames.begin(), mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints() {
  unique_lock<mutex> lock(mMutexMap);
  return vector<MapPoint*>(mspMapPoints.begin(), mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap() {
  unique_lock<mutex> lock(mMutexMap);
  return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap() {
  unique_lock<mutex> lock(mMutexMap);
  return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints() {
  unique_lock<mutex> lock(mMutexMap);
  return mvpReferenceMapPoints;
}

long unsigned int Map::GetId() { return mnId; }

boost::uuids::uuid Map::GetUuid() { return uuid; }

long unsigned int Map::GetInitKFid() {
  unique_lock<mutex> lock(mMutexMap);
  return mnInitKFid;
}

void Map::SetInitKFid(long unsigned int initKFif) {
  unique_lock<mutex> lock(mMutexMap);
  mnInitKFid = initKFif;
}

long unsigned int Map::GetMaxKFid() {
  unique_lock<mutex> lock(mMutexMap);
  return mnMaxKFid;
}

KeyFrame* Map::GetOriginKF() { return mpKFinitial; }

void Map::SetCurrentMap() { mIsInUse = true; }

void Map::SetStoredMap() { mIsInUse = false; }

void Map::clear() {
  //    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(),
  //    send=mspMapPoints.end(); sit!=send; sit++)
  //        delete *sit;

  for (set<KeyFrame*>::iterator sit = mspKeyFrames.begin(), send = mspKeyFrames.end(); sit != send; sit++) {
    KeyFrame* pKF = *sit;
    pKF->UpdateMap(static_cast<Map*>(NULL));
    //        delete *sit;
  }

  mspMapPoints.clear();
  mspKeyFrames.clear();
  mnMaxKFid = mnInitKFid;
  mbImuInitialized = false;
  mvpReferenceMapPoints.clear();
  mvpKeyFrameOrigins.clear();
  mbIMU_BA1 = false;
  mbIMU_BA2 = false;
}

bool Map::IsInUse() { return mIsInUse; }

void Map::SetBad() { mbBad = true; }

bool Map::IsBad() { return mbBad; }

void Map::ApplyScaledRotation(const Sophus::SE3f& T, const float s, const bool bScaledVel) {
  unique_lock<mutex> lock(mMutexMap);

  // Body position (IMU) of first keyframe is fixed to (0,0,0)
  Sophus::SE3f Tyw = T;
  Eigen::Matrix3f Ryw = Tyw.rotationMatrix();
  Eigen::Vector3f tyw = Tyw.translation();

  for (set<KeyFrame*>::iterator sit = mspKeyFrames.begin(); sit != mspKeyFrames.end(); sit++) {
    KeyFrame* pKF = *sit;
    Sophus::SE3f Twc = pKF->GetPoseInverse();
    Twc.translation() *= s;
    Sophus::SE3f Tyc = Tyw * Twc;
    Sophus::SE3f Tcy = Tyc.inverse();
    pKF->SetPose(Tcy);
    Eigen::Vector3f Vw = pKF->GetVelocity();
    if (!bScaledVel)
      pKF->SetVelocity(Ryw * Vw);
    else
      pKF->SetVelocity(Ryw * Vw * s);
  }
  for (set<MapPoint*>::iterator sit = mspMapPoints.begin(); sit != mspMapPoints.end(); sit++) {
    MapPoint* pMP = *sit;
    pMP->SetWorldPos(s * Ryw * pMP->GetWorldPos() + tyw);
    pMP->UpdateNormalAndDepth();
  }
  mnMapChange++;
}

void Map::SetInertialSensor() {
  unique_lock<mutex> lock(mMutexMap);
  mbIsInertial = true;
}

bool Map::IsInertial() {
  unique_lock<mutex> lock(mMutexMap);
  return mbIsInertial;
}

void Map::SetIniertialBA1() {
  unique_lock<mutex> lock(mMutexMap);
  mbIMU_BA1 = true;
}

void Map::SetIniertialBA2() {
  unique_lock<mutex> lock(mMutexMap);
  mbIMU_BA2 = true;
}

bool Map::GetIniertialBA1() {
  unique_lock<mutex> lock(mMutexMap);
  return mbIMU_BA1;
}

bool Map::GetIniertialBA2() {
  unique_lock<mutex> lock(mMutexMap);
  return mbIMU_BA2;
}

void Map::ChangeId(long unsigned int nId) { mnId = nId; }

unsigned int Map::GetLowerKFID() {
  unique_lock<mutex> lock(mMutexMap);
  if (mpKFlowerID) {
    return mpKFlowerID->mnId;
  }
  return 0;
}

int Map::GetMapChangeIndex() {
  unique_lock<mutex> lock(mMutexMap);
  return mnMapChange;
}

void Map::IncreaseChangeIndex() {
  unique_lock<mutex> lock(mMutexMap);
  mnMapChange++;
}

int Map::GetLastMapChange() {
  unique_lock<mutex> lock(mMutexMap);
  return mnMapChangeNotified;
}

void Map::SetLastMapChange(int currentChangeId) {
  unique_lock<mutex> lock(mMutexMap);
  mnMapChangeNotified = currentChangeId;
}

void Map::PreSave(std::set<GeometricCamera*>& spCams) {
  int nMPWithoutObs = 0;

  // QUESTIONABLE CODE //
  std::set<MapPoint*> tmp_mspMapPoints1;
  tmp_mspMapPoints1.insert(mspMapPoints.begin(), mspMapPoints.end());

  for (MapPoint* pMPi : tmp_mspMapPoints1) {
    if (!pMPi || pMPi->isBad())
      continue;

    if (pMPi->GetObservations().size() == 0) {
      nMPWithoutObs++;
    }
    map<KeyFrame*, std::tuple<int, int>> mpObs = pMPi->GetObservations();
    for (map<KeyFrame*, std::tuple<int, int>>::iterator it = mpObs.begin(), end = mpObs.end(); it != end; ++it) {
      if (it->first->GetMap() != this || it->first->isBad()) {
        pMPi->EraseObservation(it->first);
      }
    }
  }

  unique_lock<mutex> lock(mMutexMap);

  // Saves the id of KF origins
  mvBackupKeyFrameOriginsUuid.clear();
  mvBackupKeyFrameOriginsUuid.reserve(mvpKeyFrameOrigins.size());
  for (int i = 0, numEl = mvpKeyFrameOrigins.size(); i < numEl; ++i) {
    mvBackupKeyFrameOriginsUuid.push_back(mvpKeyFrameOrigins[i]->uuid);
  }

  // Backup of MapPoints
  mvpBackupMapPoints.clear();

  std::set<MapPoint*> tmp_mspMapPoints2;
  tmp_mspMapPoints2.insert(mspMapPoints.begin(), mspMapPoints.end());

  for (MapPoint* pMPi : tmp_mspMapPoints2) {
    if (!pMPi || pMPi->isBad())
      continue;

    mvpBackupMapPoints.push_back(pMPi);
    pMPi->PreSave(mspKeyFrames, mspMapPoints);
  }

  // Backup of KeyFrames
  mvpBackupKeyFrames.clear();
  for (KeyFrame* pKFi : mspKeyFrames) {
    if (!pKFi || pKFi->isBad())
      continue;

    mvpBackupKeyFrames.push_back(pKFi);
    pKFi->PreSave(mspKeyFrames, mspMapPoints, spCams);
  }

  mnBackupKFinitialUuiD = boost::uuids::nil_uuid();
  if (mpKFinitial) {
    mnBackupKFinitialUuiD = mpKFinitial->uuid;
  }

  mnBackupKFlowerUuiD = boost::uuids::nil_uuid();
  if (mpKFlowerID) {
    mnBackupKFlowerUuiD = mpKFlowerID->uuid;
  }
}

void Map::PostLoad(KeyFrameDatabase* pKFDB,
  ORBVocabulary* pORBVoc /*, map<long unsigned int, KeyFrame*>& mpKeyFrameId*/,
  map<unsigned int, GeometricCamera*>& mpCams, vector<KeyFrame*> existingKeyFrames,
  vector<MapPoint*> existingMapPoints) {
  unique_lock<mutex> lock(mMutexMap);

  // Add all uuid->keyframe and uuid->mappoint pairs to mpKeyFrameUuid and mpMapPointUuid so we can connect to them
  map<boost::uuids::uuid, KeyFrame*> mpKeyFrameUuid;
  for (KeyFrame* keyFrame : existingKeyFrames) {
    if (!keyFrame || keyFrame->isBad())
      continue;

    mpKeyFrameUuid[keyFrame->uuid] = keyFrame;
  }
  map<boost::uuids::uuid, MapPoint*> mpMapPointUuid;
  for (MapPoint* mapPoint : existingMapPoints) {
    if (!mapPoint || mapPoint->isBad())
      continue;

    mpMapPointUuid[mapPoint->uuid] = mapPoint;
  }

  std::copy(mvpBackupKeyFrames.begin(), mvpBackupKeyFrames.end(), std::inserter(mspKeyFrames, mspKeyFrames.begin()));

  for (MapPoint* mapPoint : mvpBackupMapPoints) {
    if (!mapPoint || mapPoint->isBad())
      continue;

    // If a mappoint already exists, use that instead of the deserialized one
    if (mpMapPointUuid.count(mapPoint->uuid) != 0) {
      mspMapPoints.insert(mpMapPointUuid[mapPoint->uuid]);
    }
    else {
      mspMapPoints.insert(mapPoint);
      mapPoint->UpdateMap(this);
      mpMapPointUuid[mapPoint->uuid] = mapPoint;
    }
  }

  for (KeyFrame* pKFi : mspKeyFrames) {
    if (!pKFi || pKFi->isBad())
      continue;

    pKFi->UpdateMap(this);
    pKFi->SetORBVocabulary(pORBVoc);
    pKFi->SetKeyFrameDatabase(pKFDB);
    mpKeyFrameUuid[pKFi->uuid] = pKFi;
  }

  // Need to make ids consistant with the other already generated maps
  // TODO: does this break loading maps from a file?
  for (MapPoint* pMPi : mspMapPoints) {
    pMPi->mnId = MapPoint::nNextId++;
  }
  for (KeyFrame* pKFi : mspKeyFrames) {
    pKFi->mnId = KeyFrame::nNextId++;
  }

  // References reconstruction between different instances
  for (MapPoint* pMPi : mspMapPoints) {
    if (!pMPi || pMPi->isBad())
      continue;

    pMPi->PostLoad(mpKeyFrameUuid, mpMapPointUuid);
  }

  for (KeyFrame* pKFi : mspKeyFrames) {
    if (!pKFi || pKFi->isBad())
      continue;

    pKFi->PostLoad(mpKeyFrameUuid, mpMapPointUuid, mpCams);
    pKFDB->add(pKFi);
  }

  if (mnBackupKFinitialUuiD != boost::uuids::nil_uuid()) {
    mpKFinitial = mpKeyFrameUuid[mnBackupKFinitialUuiD];
  }

  if (mnBackupKFlowerUuiD != boost::uuids::nil_uuid()) {
    mpKFlowerID = mpKeyFrameUuid[mnBackupKFlowerUuiD];
  }

  mvpKeyFrameOrigins.clear();
  mvpKeyFrameOrigins.reserve(mvBackupKeyFrameOriginsUuid.size());
  for (int i = 0; i < mvBackupKeyFrameOriginsUuid.size(); ++i) {
    mvpKeyFrameOrigins.push_back(mpKeyFrameUuid[mvBackupKeyFrameOriginsUuid[i]]);
  }

  mvpBackupMapPoints.clear();
}

} // namespace ORB_SLAM3
