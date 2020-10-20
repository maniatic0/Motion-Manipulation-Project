-- By Christian Oliveros and Minmin Chen
module PingPong.Player.Stig.Kinematics where

import Control.Lens (view)
import Data.Bool (bool)
import Data.Geometry hiding (head, Vector, dot)
import Data.Maybe
import GHC.Float
import Numeric.LinearAlgebra hiding (Element)
import PingPong.Model
import PingPong.Player.Stig.General
import PingPong.Player.Stig.GeometryHelpers
import Prelude hiding ((<>))

-- Inverse and Forward Kinematics

-- | Calculate the forward kinematic matrix for a joint / link
calFwdMatrix :: Element -> Element -> [[Float]]
calFwdMatrix (Joint _ a) (Link _ l) =
  [ [cos a, - sin a, l * cos a],
    [sin a, cos a, l * sin a],
    [0, 0, 1]
  ]
calFwdMatrix _ _ = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

data ArmKinematicPart
  = ArmBase
      { footParam :: R
      }
  | ArmJoint
      { translationParam :: R
      }
  | ArmBat
      { translationParam :: R
      }
  deriving (Show)

type ArmKinematic = [ArmKinematicPart]

-- | Goes from rough representation to a more refined one
fromFootArmToArmKinematic :: Float -> Arm -> ArmKinematic
fromFootArmToArmKinematic foot arm = ArmBase (float2Double foot) : processArm 0 arm
  where
    processArm :: R -> Arm -> ArmKinematic
    processArm t [Link _ t2] = [ArmBat $ t + float2Double t2]
    processArm t (Link _ t2 : as) = processArm (t + float2Double t2) as
    processArm t (Joint _ _ : as) = ArmJoint t : processArm 0 as

-- | Goes from rough representation to a more refined one.
-- As well, get the motion vector
getArmKinematicAndMotion :: Float -> Arm -> (ArmKinematic, Motion)
getArmKinematicAndMotion foot arm =
  (fromFootArmToArmKinematic foot simpleArm, getCurrentJoints simpleArm)
  where
    simpleArm = simplifyArm arm

-- | Converts a motion list to a vector of joint coordinates
motionToJointVector :: Motion -> Vector R
motionToJointVector m = fromList $ map float2Double m

-- | Converts vector of joint coordinates to a motion list
jointVectorToMotion :: Vector R -> Motion
jointVectorToMotion v = map double2Float $ toList v

-- | Translation in the X axis matrix
translateXTrans :: R -> Matrix R
translateXTrans t =
  fromLists [[1, 0, t], [0, 1, 0], [0, 0, 1]] :: Matrix R

-- | Translation in the X axis inverse matrix
translateXInvTrans :: R -> Matrix R
translateXInvTrans t =
  fromLists [[1, 0, - t], [0, 1, 0], [0, 0, 1]] :: Matrix R

-- | Rotation matrix
rotateTrans :: R -> Matrix R
rotateTrans a =
  fromLists [[c, - s, 0], [s, c, 0], [0, 0, 1]] :: Matrix R
  where
    c = cos a
    s = sin a

-- | Rotation inverse matrix
rotateInvTrans :: R -> Matrix R
rotateInvTrans a =
  fromLists [[c, s, 0], [- s, c, 0], [0, 0, 1]] :: Matrix R
  where
    c = cos a
    s = sin a

-- | Create an homogeneous 2D point
homogeneousPoint :: R -> R -> Vector R
homogeneousPoint x y = fromList [x, y, 1]

-- | Zero Point in Homogenous Coordinates
homogeneousZero :: Vector R
homogeneousZero = homogeneousPoint 0 0

-- | 2d Point to Homogeneous Coordinates
pointToHomogenousPoint :: Point 2 Float -> Vector R
pointToHomogenousPoint p = homogeneousPoint (float2Double $ view xCoord p) (float2Double $ view yCoord p)

-- | Create an homogeneous 2D vector
homogeneousVector :: R -> R -> Vector R
homogeneousVector x y = fromList [x, y, 0]

-- | X Vector
homogeneousVectorX :: Vector R
homogeneousVectorX = homogeneousVector 1 0

-- | Y Vector
homogeneousVectorY :: Vector R
homogeneousVectorY = homogeneousVector 0 1

-- | Homogeneous 2D Identity Matrix
homogeneousIdent :: Matrix R
homogeneousIdent = ident 3

data ArmKinematicPartMatrix
  = ArmBaseMatrix
      { footMatrix :: Matrix R
      }
  | ArmJointMatrix
      { translationMatrix :: Matrix R,
        jointMatrix :: Matrix R
      }
  | ArmBatMatrix
      { batMatrix :: Matrix R
      }
  deriving (Show)

type ArmKinematicMatrix = [ArmKinematicPartMatrix]

-- | Get Transform from Part Matrix
getTrans :: ArmKinematicPartMatrix -> Matrix R
getTrans (ArmBaseMatrix t) = t
getTrans (ArmJointMatrix t r) = t <> r
getTrans (ArmBatMatrix t) = t

-- | Get Inverse Transform from Part Matrix
getTransInv :: ArmKinematicPartMatrix -> Matrix R
getTransInv (ArmBaseMatrix t) = t
getTransInv (ArmJointMatrix t r) = r <> t
getTransInv (ArmBatMatrix t) = t

-- | Calculate the transforms for forward kinematics
applyForwardKinematicTrans ::
  ArmKinematic -> Vector R -> ArmKinematicMatrix
applyForwardKinematicTrans arm v = toTrans arm motion
  where
    motion = toList v :: [R]

    toTrans :: ArmKinematic -> [R] -> ArmKinematicMatrix
    toTrans (ArmBase t : as) js = (ArmBaseMatrix $ translateXTrans t <> rotateTrans (pi / 2)) : toTrans as js
    toTrans (ArmJoint t : as) (j : js) = ArmJointMatrix (translateXTrans t) (rotateTrans j) : toTrans as js
    toTrans [ArmBat t] [] = [ArmBatMatrix $ translateXTrans t]

-- | Calculate the inverse transforms for forward kinematics
applyForwardKinematicTransInv ::
  ArmKinematic -> Vector R -> ArmKinematicMatrix
applyForwardKinematicTransInv arm v = reverse $ toTrans arm motion
  where
    motion = toList v :: [R]

    toTrans :: ArmKinematic -> [R] -> ArmKinematicMatrix
    toTrans (ArmBase t : as) js = (ArmBaseMatrix $ rotateInvTrans (pi / 2) <> translateXInvTrans t) : toTrans as js
    toTrans (ArmJoint t : as) (j : js) = ArmJointMatrix (translateXInvTrans t) (rotateInvTrans j) : toTrans as js
    toTrans [ArmBat t] [] = [ArmBatMatrix $ translateXInvTrans t]

-- | Apply Forward Kinematic Transformations
applyForwardKinematicMatrixTrans :: ArmKinematicMatrix -> Matrix R
applyForwardKinematicMatrixTrans = foldr ((<>) . getTrans) homogeneousIdent

-- | Apply Forward Kinematic Inverse Transformations (the list must go from end effecto to base)
applyForwardKinematicMatrixTransInv :: ArmKinematicMatrix -> Matrix R
applyForwardKinematicMatrixTransInv = foldr ((<>) . getTransInv) homogeneousIdent

-- | Compress the transforms of a Forward Kinematic Arm to only the joints and the end effector
compressForwardKinematicsJointsTrans :: ArmKinematicMatrix -> ArmKinematicMatrix
compressForwardKinematicsJointsTrans a = compress a homogeneousIdent
  where
    compress :: ArmKinematicMatrix -> Matrix R -> ArmKinematicMatrix
    compress [ArmBatMatrix t] m = [ArmBatMatrix (m <> t)]
    compress (ArmJointMatrix t r : as) m = ArmJointMatrix (m <> t) r : compress as homogeneousIdent
    compress (a : as) m = compress as (m <> getTrans a)

-- | Compress the transforms of a Forward Kinematic Arm to only the joints and the end base
compressForwardKinematicsJointsInvTrans :: ArmKinematicMatrix -> ArmKinematicMatrix
compressForwardKinematicsJointsInvTrans a = compress a homogeneousIdent
  where
    compress :: ArmKinematicMatrix -> Matrix R -> ArmKinematicMatrix
    compress [ArmBaseMatrix t] m = [ArmBaseMatrix (t <> m)]
    compress (ArmJointMatrix t r : as) m = ArmJointMatrix (t <> m) r : compress as homogeneousIdent
    compress (a : as) m = compress as (getTransInv a <> m)

-- | Calculate the 2D Homogeneous Jacobian of an arm [J^T|0]^T .
-- fwdMatTrans : Kinematic Forward Transforms (From base to end effector).
-- fwdMatTransInv : Kinematic Forward Inverse Transforms (From end effector to base).
-- xLocal : Position of Tool on End Effector Local Coordinates
calculateJacobian ::
  ArmKinematicMatrix -> ArmKinematicMatrix -> Vector R -> Matrix R
calculateJacobian fwdMatTrans fwdMatTransInv xLocal = jH
  where
    -- Compressed form guarantees that the links were applied to the joints
    fwdTransCompressed = compressForwardKinematicsJointsTrans fwdMatTrans
    fwdTransInvCompressed = compressForwardKinematicsJointsInvTrans fwdMatTransInv

    -- Derivative of a revolute Joint
    revoluteDeriv = fromLists [[0, -1, 0], [1, 0, 0], [0, 0, 0]] :: Matrix R

    rollingTransInv :: ArmKinematicMatrix -> Matrix R -> [Matrix R]
    rollingTransInv [ArmBaseMatrix t] m = [t <> m]
    rollingTransInv (a : as) m = roll : rollingTransInv as roll
      where
        roll = getTransInv a <> m
    calculate ::
      ArmKinematicMatrix -> Matrix R -> [Matrix R] -> [Vector R]
    calculate [ArmBatMatrix _] _ [_] = [] -- We reach the bat, we no longer need to process
    calculate (j@(ArmJointMatrix _ _) : as) m (endToJ : mIs) =
      (baseToJ <> revoluteDeriv <> endToJ) #> xLocal : calculate as baseToJ mIs
      where
        baseToJ = m <> getTrans j

    -- Jacobian in this form: [J^T|0]^T there is a row of 0s at the end due to 2d homogeneous coordinates
    jH =
      fromColumns $
        calculate fwdTransCompressed homogeneousIdent (rollingTransInv fwdTransInvCompressed homogeneousIdent)

-- | Get the Jacobian from the Homogeneous Jacobian (Note this doesn't work for the inverse of the Jacobian)
getJacobianFromHomogeneousJacobian :: Matrix R -> Matrix R
getJacobianFromHomogeneousJacobian jH = jH ?? (DropLast 1, All)

-- | Newton Raphson Threshold
newtonRaphsonThreshold :: (Num r, Ord r, Fractional r) => r -> r -> r
newtonRaphsonThreshold = threshold 0.0000001

-- | Calculates a Newton-Raphson IK step (if singular it fails). Returns the new Joints and its error agains target
-- a : Arm Description.
-- q : Current Joints.
-- xLocal : Tool in End-Effector Local Coordinates
-- xTargetGlobal : Target in Global Coordinates
newtonRaphsonStep ::
  ArmKinematic ->
  Vector R ->
  Vector R ->
  Vector R ->
  Maybe (Vector R, R)
newtonRaphsonStep a q xLocal xTargetGlobal
  | singular = Nothing
  | otherwise = Just (qn, norm_2 eN)
  where
    -- Forward Transforms
    fwdT = applyForwardKinematicTrans a q
    fwdTI = applyForwardKinematicTransInv a q
    jH = calculateJacobian fwdT fwdTI xLocal

    -- Bat Global Position
    batGlobal = applyForwardKinematicMatrixTrans fwdT #> xLocal

    -- Error
    e = xTargetGlobal - batGlobal

    -- Drop Homogeneous part
    er = subVector 0 2 e
    j = getJacobianFromHomogeneousJacobian jH

    -- Calculate delta q
    dq = - pinv j #> er

    dqNorm = norm_2 dq

    qnUnormalized = q + dq
    -- Re normalizes angles for more accuracy
    qn = fromList $ map normalizeAngle $ toList qnUnormalized

    -- New Bat Position
    batN = applyForwardKinematicMatrixTrans (applyForwardKinematicTrans a qn) #> xLocal

    -- New Error
    eN = xTargetGlobal - batN

    -- If the move was singular
    singular = rank jH < 2 && newtonRaphsonThreshold 0 dqNorm == 0

-- | Maximum Newton Raphson Step
newtonRaphsonMaxStep :: Int
newtonRaphsonMaxStep = 10000

-- | Really Bad Step
newtonRaphsonBadStep :: forall a. Floating a => a
newtonRaphsonBadStep = 2

-- | When to do a random Restart Newton Raphson Step
newtonRaphsonMaxRandomRestartStep :: Int
newtonRaphsonMaxRandomRestartStep = round (fromIntegral newtonRaphsonMaxStep / 10)

-- | NewtonRaphson Loop Iteration
newtonRaphsonIKIter ::
  Int ->
  Int ->
  ArmKinematic ->
  (Vector R, Vector R) ->
  Vector R ->
  (Vector R, R) ->
  (Vector R, R)
newtonRaphsonIKIter i j a (xLocal, xTargetGlobal) q (qBest, eBest)
  | i >= newtonRaphsonMaxStep = (qBest, eBest)
  | newtonRaphsonThreshold 0 eBest == 0 = (qBest, eBest)
  | singular = newtonRaphsonIKIter (i + 1) 0 a (xLocal, xTargetGlobal) qR (qBest, eBest)
  | needReset = newtonRaphsonIKIter (i + 1) 0 a (xLocal, xTargetGlobal) qR (qBest, eBest)
  | eN < eBest = newtonRaphsonIKIter (i + 1) (j + 1) a (xLocal, xTargetGlobal) qN (qN, eN)
  | otherwise = newtonRaphsonIKIter (i + 1) (j + 1) a (xLocal, xTargetGlobal) qN (qBest, eBest)
  where
    -- Random Vector
    qSize = size q
    pseudoInt = round $ fromIntegral i ** (dot q qBest * bool (eBest + 1) (1 / eBest) (eBest < 1 && 0 < eBest))
    qR = pi / 2 * (2 * randomVector pseudoInt Uniform qSize - 1)

    -- Perform the step
    step = newtonRaphsonStep a q xLocal xTargetGlobal

    singular = case step of
      Nothing -> True
      Just _ -> False

    (qN, eN) = fromJust step

    needReset =
      ( eN >= ((eBest / (1 + fromIntegral j)) * newtonRaphsonBadStep)
          || (newtonRaphsonThreshold 0 (norm_2 (qN - q)) == 0)
      )
        && j >= newtonRaphsonMaxRandomRestartStep

-- | Newton Raphson IK Algorithm
newtonRaphsonIK ::
  ArmKinematic ->
  (Vector R, Vector R) ->
  Vector R ->
  (Vector R, R)
newtonRaphsonIK a (xLocal, xTargetGlobal) q = newtonRaphsonIKIter 0 0 a (xLocal, xTargetGlobal) q (q, eNorm)
  where
    -- Forward Transforms
    fwdT = applyForwardKinematicTrans a q

    -- Bat Global Position
    batGlobal = applyForwardKinematicMatrixTrans fwdT #> xLocal

    -- Error
    e = xTargetGlobal - batGlobal
    eNorm = norm_2 e

-- Calculates newton raphson step to approximate acos
newtonRaphsonAcosStep :: Int -> Int -> Double -> (Double, Double) -> Double -> (Double, Double)
newtonRaphsonAcosStep i j q (qB, eB) t
  | i == newtonRaphsonMaxStep = (qB, eB)
  | newtonRaphsonThreshold 0 eB == 0 = (qB, eB)
  | isSingular || needReset = newtonRaphsonAcosStep (i + 1) 0 qR (qB, eB) t
  | eN < eB = newtonRaphsonAcosStep (i + 1) (j + 1) qN (qN, eN) t
  | otherwise = newtonRaphsonAcosStep (i + 1) (j + 1) qN (qB, eB) t
  where
    -- Calculate derivative and function
    f = t - cos q
    fDeriv = sin q

    -- Check if we are in a singularity
    isSingular = newtonRaphsonThreshold 0 fDeriv == 0

    -- Step
    qN = normalizeAngle $ q - f / fDeriv

    -- New Error
    eN = t - cos qN

    -- Random Restarts
    needReset = eN >= eB * newtonRaphsonBadStep && j >= newtonRaphsonMaxRandomRestartStep

    pseudoInt = round $ fromIntegral i ** (q * qB * bool (eB + 1) (1 / eB) (eB < 1 && 0 < eB))
    randNum = atIndex (randomVector pseudoInt Uniform 1) 0
    qR = pi / 2 * (2 * randNum - 1)

-- | Approximate Acos with newton raphson
newtonRaphsonAcos :: Double -> Double -> (Double, Double)
newtonRaphsonAcos q t = newtonRaphsonAcosStep 0 0 qN (qN, eN) t
  where
    qN = normalizeAngle q
    eN = t - cos qN