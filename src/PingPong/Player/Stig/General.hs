-- By Christian Oliveros and Minmin Chen
module PingPong.Player.Stig.General where

import Data.Bool (bool)
import PingPong.Model
import Graphics.Gloss

-- General Helpers

-- | Check that the arm elements are valid
checkArmElements :: Arm -> Arm
checkArmElements [] = error "The Empty Arm is no valid"
checkArmElements l@[Link _ len] = bool (error "Bat has wrong length it must be 0.1") l (len == 0.1)
checkArmElements [Joint _ _] = error "A Joint can't be the end of the Arm"
checkArmElements (j@(Joint _ _) : xs@(Link _ _ : _)) = j : checkArmElements xs
checkArmElements (Joint _ _ : (Joint _ _ : _)) = error "Two Joints together"
checkArmElements (Link _ _ : (Link _ _ : _)) = error "Two Links together"
checkArmElements (l@(Link _ len) : xs@(Joint _ _ : _)) =
  bool (error "Arm has a Link with wrong length [.1, .5]") (l : checkArmElements xs) (0.1 <= len && len <= 0.5)

-- | Check that the arm is valid
checkArm :: Arm -> Arm
checkArm ar =
  bool (error "Wrong number of Joints [2..5]") (checkArmElements ar) (2 <= jointCount && jointCount <= 5)
  where
    jointCount = foldl f 0 ar
      where
        f :: Integer -> Element -> Integer
        f n (Joint _ _) = n + 1
        f n (Link _ _) = n

-- | Get the current Joint parameters as a Motion
getCurrentJoints :: Arm -> Motion
getCurrentJoints [] = []
getCurrentJoints (Joint _ rad : xs) = rad : getCurrentJoints xs
getCurrentJoints (Link _ _ : xs) = getCurrentJoints xs

-- | Motion Velocity Limit
motionLimit :: Float
motionLimit = 1.0

-- | Apply Motion limits to avoid simulation problems
applyMotionLimits :: Motion -> Motion
applyMotionLimits = map f
  where
    f x = min motionLimit $ max (- motionLimit) x

-- | Simplify arm with adjacent joints or links
simplifyArm :: Arm -> Arm
--simplifyArm (Joint color a1 : (Joint _ a2 : xs)) = (Joint color (a1 + a2)) : xs
simplifyArm [] = []
simplifyArm (Link color l1 : Link _ l2 : xs) = Link color (l1 + l2) : xs
simplifyArm (x : xs) = x : simplifyArm xs

paleBlue :: Color
paleBlue = makeColor 0.5 0.5 0.6 1

red :: Color
red = makeColor 1 0 0 1

hotPink :: Color
hotPink = makeColor 1.0 0.2 0.7 1

-- End of General Helpers