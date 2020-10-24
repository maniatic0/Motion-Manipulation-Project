-- By Christian Oliveros and Minmin Chen
module PingPong.Player.Stig.Fabrik where

import Data.Bool (bool)
import PingPong.Model
import PingPong.Player.Stig.General
import PingPong.Player.Stig.GeometryHelpers
import Data.Geometry hiding (head)
import Control.Lens (view, (&), (.~), (^.))
import Data.Ext
import Debug.Trace
import GHC.Float

-- | FABRIK 2D reach
reach :: (Num r, Floating r, Ord r) => Point 2 r -> Point 2 r -> Point 2 r -> (Point 2 r, Point 2 r)
reach p1 p0 target = (target, tailP)
    where
        lDist = norm (p0 .-. p1)
        (nlDir, _) = normalizeVector $ p0 .-. target
        tailP = p1 .+^ (nlDir ^* lDist)

-- | FABRIK internal iteration
fabrikIter :: (Num r, Floating r, Ord r) => [Point 2 r] -> Point 2 r -> [Point 2 r]
fabrikIter [_] tgt = [tgt]
fabrikIter (p1:p0:rest) tgt = newP1 : fabrikIter (p0:rest) newTgt
    where
        (newP1, newTgt) = reach p1 p0 tgt

-- | Perform a FABRIK step, arm must be from end effector to base
fabrikStep :: (Num r, Floating r, Ord r) => [Point 2 r] -> Point 2 r -> ([Point 2 r], r)
fabrikStep arm tgt = (newArm, err)
    where
        base = last arm
        fbArm = fabrikIter arm tgt
        ikArm = fabrikIter (reverse fbArm) base
        newArm = reverse ikArm
        err = norm $ tgt .-. head newArm

-- | Fabrik Threshold, arm must be from end effector to base
fabrikThreshold :: (Num r, Ord r, Fractional r) => r -> r -> r
fabrikThreshold = threshold 0.001

fabrikMaxIter :: Int
fabrikMaxIter = 1000

-- | Apply the FABRIK algorithm, arm must be from end effector to base
fabrikAlgo :: (Num r, Floating r, Ord r, Show r) => Int -> [Point 2 r] -> Point 2 r -> ([Point 2 r], r)
fabrikAlgo iter arm tgt
    | (iter + 1) >= fabrikMaxIter = trace ("Max Iter " ++ show err) (newArm, err)
    |  fabrikThreshold 0 err == 0 = trace ("Reached " ++ show err)(newArm, err)
    | otherwise = fabrikAlgo (iter + 1) newArm tgt
    where
        (newArm, err) = fabrikStep arm tgt

-- | Apply the FABRIK algorithm, arm must be from end effector to base
fabrikApply :: (Num r, Floating r, Ord r, Show r) => [Point 2 r] -> Point 2 r -> ([Point 2 r], r) 
fabrikApply = fabrikAlgo 0

-- | Convert fromn foot arm space to the fabrik space. Note that arm mut go from base to end effector
toFabrikSpace :: Float -> Arm -> Point 2 Float -> ([Point 2 Double], Point 2 Double)
toFabrikSpace foot arm tgt = (newArm, newTgt)
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

        -- We centered space at (0,0), which is the base of the robot (if it uses first link, then we moved that as well)
        base = Point2 0 0

        -- Create Fabrik arm, it goes from end effector to base
        newArm = reverse $ base:armToFabrik base (if startsLink then tail simpleArm else simpleArm)

        -- | Get Link Length
        getLinkLength :: Element -> Double
        getLinkLength (Link _ l) = float2Double l

        -- | Creates the points for Fabrik (goes from base to end effector)
        armToFabrik :: Point 2 Double -> Arm -> [Point 2 Double]
        armToFabrik b [Link _ l] = [b .+^ Vector2 0 (float2Double l)]
        armToFabrik b (Joint _ _:rest) = armToFabrik b rest
        armToFabrik b (Link _ l:rest) = nB : armToFabrik nB rest
            where
                nB = b .+^ Vector2 0 (float2Double l)

-- | Converts an arm from fabrik space to motion space
fabrikSpaceToMotion :: [Point 2 Double] -> Motion
fabrikSpaceToMotion arm = fabrikSpaceToMotionInternal (Vector2 0 1) armInv
    where
        armInv = reverse arm

        -- | Converts a fabrik arm from base to end effector to the motion representation
        fabrikSpaceToMotionInternal :: Vector 2 Double -> [Point 2 Double] -> Motion
        fabrikSpaceToMotionInternal _ [_] = []
        fabrikSpaceToMotionInternal v (p0:p1:rest) = double2Float angle : fabrikSpaceToMotionInternal nV (p1:rest)
            where
                nV = p1 .-. p0
                angle = angleVectorPrecise3 v nV 

-- | Fabrik to Point
fabrikToPoint :: Float -> Arm -> Point 2 Float -> (Motion, Double)
fabrikToPoint foot arm tgt = (m, err)
    where
        (newArm, newTgt) = toFabrikSpace foot arm tgt
        (tgtArm, err) = fabrikApply newArm newTgt
        m = fabrikSpaceToMotion tgtArm
