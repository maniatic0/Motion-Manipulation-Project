module PingPong.Simulation.Recording where

import PingPong.Model
import PingPong.Draw.Rasterific
import PingPong.Simulation

import Codec.Picture( PixelRGBA8( .. ), writePng )
import Graphics.Rasterific
import Graphics.Rasterific.Texture
import Graphics.Text.TrueType( Font, loadFontFile )

import System.Directory
import System.Process

import Data.Char


-- import Graphics.Gloss (Display (InWindow), Picture, Color, white)
-- import Graphics.Gloss.Export.Image

frameRate :: Float
frameRate = 50

frameCount :: Int
frameCount = 10 -- 00

frameDuration :: Float
frameDuration = 1 / frameRate

play :: Player -> Player -> IO ()
play ip1 ip2 = do
  putStrLn $ "[" ++ name ip1 ++ " versus " ++ name ip2 ++ "]"
  Right font <- loadFontFile "Montserrat-Bold.ttf"
  pics <- record frameCount font initialState
  export pics (filter isAlphaNum (name ip1) ++ "-" ++ filter isAlphaNum (name ip2))
  where
    initialState :: State
    initialState = State ip2 ip1 startBall

record :: Int -> Font -> State -> IO [Drawing PixelRGBA8 ()]
record 0 _    _  = return []
record n font os = do
  (ns, pic) <- step font os
  pics      <- record (n - 1) font ns 
  return $ pic : pics

step :: Font -> State -> IO (State, Drawing PixelRGBA8 ())
step font os = do
  let pic = drawState font os
  ns <- update frameDuration os
  return (ns, pic)




export :: [Drawing PixelRGBA8 ()] -> String -> IO ()
export pics matchName = do
  createDirectoryIfMissing True "recording/frame/"
  putStr "recording"
  sequence_ $ zipWith exportFrame [1..] pics
  putStrLn "done!"
  callCommand $ "ffmpeg -y -r " ++ show frameRate ++ " -f image2 -s 1920x1080 -i recording/frame/%d.png -vcodec libx264 -crf 25  -pix_fmt yuv420p recording/" ++ matchName ++ ".mp4"
  removeDirectoryRecursive "recording/frame/"

exportFrame :: Int -> Drawing PixelRGBA8 () -> IO ()
exportFrame i pic = do
  let white = PixelRGBA8 255 255 255 255
      img = renderDrawing 1920 1080 white pic
  writePng ("recording/frame/" ++ show i ++ ".png") img
  putStr $ if (i `mod` round frameRate == 0) then "|" else "."





