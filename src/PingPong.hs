module PingPong where

import PingPong.Model
import PingPong.Player
import PingPong.Player.MrStiff
import PingPong.Player.MsWavy
import PingPong.Player.Caller
import PingPong.Player.Stig
import PingPong.Simulation

main :: IO ()
main = play msWavy stig