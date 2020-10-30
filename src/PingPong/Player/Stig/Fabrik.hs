-- By Christian Oliveros and Minmin Chen
module PingPong.Player.Stig.Fabrik where

import Control.Lens (view, (&), (.~), (^.))
import Data.Bool (bool)
import Data.Ext
import Data.Geometry hiding (head)
import Debug.Trace
import GHC.Float
import PingPong.Model
import PingPong.Player.Stig.General
import PingPong.Player.Stig.GeometryHelpers

-- | Get Link Length
getLinkLength :: Element -> Double
getLinkLength (Link _ l) = float2Double l

-- | Fabrik Threshold, arm must be from end effector to base
fabrikThreshold :: (Num r, Ord r, Fractional r) => r -> r -> r
fabrikThreshold = threshold 0.000000001

-- | FABRIK 2D reach
reach :: (Num r, Floating r, Ord r, Show r) => r -> Point 2 r -> Point 2 r -> Point 2 r -> (Point 2 r, Point 2 r)
reach l _ p0 target = (target, tailP)
  where
    (nlDir, nNorm) = normalizeVectorThreshold fabrikThreshold $ p0 .-. target
    tailP = target .+^ bool (Vector2 l 0) (nlDir ^* l) (nNorm > 0) -- the bool is in the case p0 is at target

-- | FABRIK internal iteration
fabrikIter :: (Num r, Floating r, Ord r, Show r) => [r] -> [Point 2 r] -> Point 2 r -> [Point 2 r]
fabrikIter [] [_] tgt = [tgt]
fabrikIter (l : ls) (p1 : p0 : rest) tgt = newP1 : fabrikIter ls (p0 : rest) newTgt
  where
    (newP1, newTgt) = reach l p1 p0 tgt

-- | Perform a FABRIK step, arm must be from end effector to base
fabrikStep :: (Num r, Floating r, Ord r, Show r) => [r] -> [Point 2 r] -> Point 2 r -> ([Point 2 r], r)
fabrikStep links arm tgt = (newArm, err)
  where
    base = last arm
    fbArm = fabrikIter links arm tgt
    ikArm = fabrikIter (reverse links) (reverse fbArm) base
    newArm = reverse ikArm
    err = norm $ tgt .-. head newArm

-- | Fabrik max iteration
fabrikMaxIter :: Int
fabrikMaxIter = 100

-- | Apply the FABRIK algorithm, arm must be from end effector to base
fabrikAlgo :: (Num r, Floating r, Ord r, Show r) => Int -> [r] -> [Point 2 r] -> Point 2 r -> ([Point 2 r], r)
fabrikAlgo iter links arm tgt
  | (iter + 1) >= fabrikMaxIter = (newArm, err)
  | fabrikThreshold 0 err == 0 = (newArm, err)
  | otherwise = fabrikAlgo (iter + 1) links newArm tgt
  where
    (newArm, err) = fabrikStep links arm tgt

-- | Apply the FABRIK algorithm, arm must be from end effector to base
fabrikApply :: (Num r, Floating r, Ord r, Show r) => [r] -> [Point 2 r] -> Point 2 r -> ([Point 2 r], r)
fabrikApply = fabrikAlgo 0

-- Center at the base of the arm which we put at (0,0), move space there
pointToFabrikSpace :: Float -> Arm -> Point 2 Float -> Point 2 Double
pointToFabrikSpace foot arm tgt = newTgt
  where
    -- Simplify redundant links
    simpleArm = simplifyArm arm

    -- We have to see if we start with a link or a joint
    armHead = head simpleArm
    startsLink = not $ isJoint armHead

    -- if we start
    -- We are centered at the base of the arm which we put at (0,0), move space there
    tgtDouble = Point2 (float2Double $ view xCoord tgt) (float2Double $ view yCoord tgt)
    newTgt = tgtDouble .-^ Vector2 (float2Double foot) (bool 0 (getLinkLength armHead) startsLink)

-- | Convert fromn foot arm space to the fabrik space. Note that arm mut go from base to end effector
toFabrikSpace :: Float -> Arm -> Point 2 Float -> ([Point 2 Double], [Double], Point 2 Double)
toFabrikSpace foot arm tgt = (newArm, links, newTgt)
  where
    -- Simplify redundant links
    simpleArm = simplifyArm arm

    -- We have to see if we start with a link or a joint
    armHead = head simpleArm
    startsLink = not $ isJoint armHead

    -- if we start
    -- We are centered at the base of the arm which we put at (0,0), move space there
    newTgt = pointToFabrikSpace foot arm tgt

    -- We centered space at (0,0), which is the base of the robot (if it uses first link, then we moved that as well)
    base = Point2 0 0

    -- Create Fabrik arm, it goes from end effector to base
    armToUse = if startsLink then tail simpleArm else simpleArm
    newArm = reverse $ base : armToFabrik base armToUse

    -- We want link lengths
    links = reverse $ map getLinkLength $ filter (not . isJoint) armToUse

    armToFabrik :: Point 2 Double -> Arm -> [Point 2 Double]
    armToFabrik b [Link _ l] = [b .+^ Vector2 0 (float2Double l)]
    armToFabrik b (Joint _ _ : rest) = armToFabrik b rest
    armToFabrik b (Link _ l : rest) = nB : armToFabrik nB rest
      where
        nB = b .+^ Vector2 0 (float2Double l)

-- | Converts an arm from fabrik space to motion space
fabrikSpaceToMotion :: [Point 2 Double] -> Motion
fabrikSpaceToMotion [] = []
fabrikSpaceToMotion arm = fabrikSpaceToMotionInternal (Vector2 0 1) armInv
  where
    armInv = reverse arm

    fabrikSpaceToMotionInternal :: Vector 2 Double -> [Point 2 Double] -> Motion
    fabrikSpaceToMotionInternal _ [_] = []
    fabrikSpaceToMotionInternal v (p0 : p1 : rest) = double2Float angle : fabrikSpaceToMotionInternal nV (p1 : rest)
      where
        nV = p1 .-. p0
        angle = angleVectorPrecise2Signed v nV

-- | Fabrik to Point
fabrikToPoint :: Float -> Arm -> Point 2 Float -> (Motion, Double)
fabrikToPoint foot arm tgt = (m, err)
  where
    (newArm, links, newTgt) = toFabrikSpace foot arm tgt
    (tgtArm, err) = fabrikApply links newArm newTgt
    m = fabrikSpaceToMotion tgtArm

-- | Fabrik to Segment
fabrikToSegment :: Float -> Arm -> LineSegment 2 () Float -> (Motion, Double, Double)
fabrikToSegment foot arm lTgt
  | isOnlyBat = (mOnlyBat, eBOnlyBat, errOnlyBat)
  | otherwise = (m, errBatless, errBat)
  where
    -- Target Info
    startPoint = lTgt ^. (start . core)
    endPoint = lTgt ^. (end . core)

    simplerArmInv = reverse $ simplifyArm arm
    bat = head simplerArmInv
    batlessArm = reverse $ tail $ tail simplerArmInv -- We were promised no dual joints

    -- Check if the bat is only an arm
    isOnlyBat = null batlessArm

    eBOnlyBat = norm $ pointFloat2Double startPoint .-. Point2 (float2Double foot) 0
    (newArmOnlyBat, linksOnlyBat, newTgtOnlyBat) = toFabrikSpace foot arm endPoint
    (tgtArmOnlyBat, errOnlyBat) = fabrikApply linksOnlyBat newArmOnlyBat newTgtOnlyBat
    mOnlyBat = fabrikSpaceToMotion tgtArmOnlyBat

    (newArmBatless, linksBatless, newTgtBatless) = toFabrikSpace foot batlessArm startPoint
    (tgtArmBatless, errBatless) = fabrikApply linksBatless newArmBatless newTgtBatless

    batStart = head tgtArmBatless
    target = pointToFabrikSpace foot arm endPoint

    (_, newBatEnd) = reach (getLinkLength bat) batStart target batStart
    m = fabrikSpaceToMotion (newBatEnd : tgtArmBatless)

    errBat = norm $ target .-. newBatEnd
