from inc_noesis import *
import noesis
import rapi
from GravityRush_common import *
from EdgeLib20 import *

debug = False
printLog = False
export_json = False #Set it to False to disable, or as export location string 
global_scale = 100

def registerNoesisTypes():
    handle = noesis.register('Gravity Rush 2 MOT', '.mot')
    noesis.setHandlerTypeCheck(handle, noepyCheckType)
    noesis.setHandlerLoadModel(handle, noepyLoadModel)
    if debug:
        noesis.logPopup()
    return 1


def noepyCheckType(data):
    file = NoeBitStream(data)
    if len(data) < 4:
        return 0
    header = file.readBytes(4).decode('ASCII').rstrip("\0")
    if header == '1DMG':
        return 1
    return 0


def noepyLoadModel(data, mdlList):
  fileList = splitMotFile(data)
  mdl = NoeModel()

  #Processing Skeleton
  skel_data = fileList['.skel']
  pSkel = EdgeAnimationSkeleton(skel_data)
  skeleton = ExtractSkeleton(pSkel)
  if printLog:
    printSkelLog(pSkel, skeleton)
  noeSkeleton = loadSkeleton(skeleton)
  mdl.setBones(noeSkeleton)

  #Process Animation
  if ".anim" in fileList:
    anim_data = fileList['.anim']
    pAnim = EdgeAnimAnimation(anim_data)
    compressed_anim = ReadAnimation(pAnim)
    animation = DecompressAnimation(compressed_anim)
    if printLog:
      printAnimLog(pAnim, compressed_anim, animation, noeSkeleton)
    noeAnimation = loadAnimation(animation, noeSkeleton)
    mdl.setAnims([noeAnimation])

    if export_json:
      try:
        os.makedirs(os.path.dirname(export_json), exist_ok=True) #Not sure why exist_ok doesn't work
      except:
        pass
      userChannels = {}
      for channelIndex in range(len(animation.m_userChannelAnimations)):
        channel = animation.m_userChannelAnimations[channelIndex]
        
        if len(channel.m_animation) > 0:
          channelData = {}
          for i in range(len(channel.m_animation)):
            key = channel.m_animation[i]
            keyData = {}
            keyData["Value"] = key.m_keyData[0]
            # keyData["TimeStamp"] = key.m_keyTime
            if strAnimationKeyframeFlags(key):
              keyData["Key"] = strAnimationKeyframeFlags(key)
            # channelData.append(keyData)
            # print(key.m_keyTime * pAnim.sampleFrequency)
            # print(round(key.m_keyTime * pAnim.sampleFrequency))
            # print(str(round(key.m_keyTime * pAnim.sampleFrequency)))
            # print()
            channelData[round(key.m_keyTime * pAnim.sampleFrequency)] = keyData
          userChannels[channelIndex] = channelData



      with open("%s\%s.json" % (export_json, rapi.getInputName().split('\\')[-1].split('.')[0]), 'w') as outfile:
        jsonData = {}
        jsonData["FPS"] = round(pAnim.sampleFrequency)
        jsonData["Channel"] = userChannels
        json.dump(jsonData, outfile, indent=4, separators=(',', ': '), sort_keys=True)
  
  mdlList.append(mdl)
  return 1

def loadSkeleton(skeleton):
  joints = []
  for jointIndex in range(skeleton.m_numJoints):
    jointName = getNameFromHash(skeleton.m_jointNameHashes[jointIndex])
    parentJointIndex = skeleton.m_parentIndices[jointIndex]
    rotation = NoeQuat(skeleton.m_basePose[jointIndex].m_rotation)
    translation = NoeVec3(skeleton.m_basePose[jointIndex].m_translation[:3]) * NoeVec3([global_scale, global_scale, global_scale])
    scale = NoeVec3(skeleton.m_basePose[jointIndex].m_scale[:3])
    jointMat = rotation.toMat43(transposed=1)
    jointMat[3] = translation
    jointMat *- scale
    joints.append(NoeBone(jointIndex, jointName, jointMat, None, parentJointIndex))

  for joint in joints:
      joint.setMatrix(joint.getMatrix() * joints[joint.parentIndex].getMatrix())

  return joints

def loadAnimation(animation, noeBones):
  kfBones = []
  for jointIndex in range(len(animation.m_jointAnimations)):
    joint = animation.m_jointAnimations[jointIndex]
    kfBone = NoeKeyFramedBone(jointIndex)
    
    rotation = []
    for key in joint.m_rotationAnimation:
      rotation.append(NoeKeyFramedValue(key.m_keyTime, NoeQuat(key.m_keyData).transpose()))
    if rotation != []:
      rotation.append(NoeKeyFramedValue(key.m_keyTime + 1/30, NoeQuat(key.m_keyData).transpose())) #Noesis BUG, First frame is duplicated and last frame is dropped
    kfBone.setRotation(rotation)

    translation = []
    for key in joint.m_translationAnimation:
      translation.append(NoeKeyFramedValue(key.m_keyTime, NoeVec3(key.m_keyData[:3]) * NoeVec3([global_scale, global_scale, global_scale])))
    if translation != []:
      translation.append(NoeKeyFramedValue(key.m_keyTime + 1/30, NoeVec3(key.m_keyData[:3]) * NoeVec3([global_scale, global_scale, global_scale])))
    kfBone.setTranslation(translation)

    scale = []
    for key in joint.m_scaleAnimation:
      scale.append(NoeKeyFramedValue(key.m_keyTime, NoeVec3(key.m_keyData[:3])))
    if scale != []:
      scale.append(NoeKeyFramedValue(key.m_keyTime + 1/30, NoeVec3(key.m_keyData[:3])))
    kfBone.setScale(scale, noesis.NOEKF_SCALE_TRANSPOSED_VECTOR_3)

    kfBones.append(kfBone)
  anim = NoeKeyFramedAnim("unknown", noeBones, kfBones, 30.0)
  return anim

def splitMotFile(data):
  bs = NoeBitStream(data)
  bs.seek(0x10, NOESEEK_ABS)
  fileCount = bs.readInt()

  offsetToFile = []
  filetype = []
  bs.seek(0x20, NOESEEK_ABS)
  for i in range(0, fileCount):
    currentOffset = bs.tell()
    bs.seek(currentOffset + bs.readUInt(), NOESEEK_ABS)
    filetype.append(noeStrFromBytes(bs.readBytes(6)).split('\x00')[0])
    bs.seek(0x10 + currentOffset, NOESEEK_ABS)
    offsetToFile.append(
        bs.tell() + bs.readUInt())
    bs.seek(0x0C, NOESEEK_REL)

  offsetToFile, filetype = zip(*sorted(zip(offsetToFile, filetype)))

  fileList = {}

  for i in range(0, fileCount):
    if i == fileCount - 1:
        targetData = data[offsetToFile[i]: len(data)]
    else:
        targetData = data[offsetToFile[i]: offsetToFile[i+1]]
    extension = filetype[i]
    fileList[extension] = targetData

  return fileList
