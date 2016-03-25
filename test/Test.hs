{-# LANGUAGE TypeOperators #-}
module Main where

import Control.Concurrent
import Linear.Extra
import Physics.Bullet

dt :: Float
dt = 1 / 60

main :: IO ()
main = do
    dynamicsWorld  <- createDynamicsWorld DynamicsWorldConfig{ dwGravity = -9.81 }

    -- ground plane
    groundShape <- createStaticPlaneShape (0 :: Float)
    ground <- addRigidBody dynamicsWorld 0 groundShape mempty{ rbRotation    = axisAngle (V3 1 0 0) (-pi/2)
                                                             , rbMass        = 0
                                                             }

    ballShape <- createSphereShape (1 :: Float)
    ball <- addRigidBody dynamicsWorld 1 ballShape mempty{ rbPosition = V3 0 10 0
                                                         , rbMass = 1
                                                         }

    let loop = do stepSimulationWithTimestep dynamicsWorld (realToFrac dt) 10 dt

                  bst <- getBodyState ball :: IO (V3 Float, Quaternion Float)
                  print bst

                  threadDelay (floor $ dt * 1000000)
                  loop

    loop
    putStrLn "OK"
