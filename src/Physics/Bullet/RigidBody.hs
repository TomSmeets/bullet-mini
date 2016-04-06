{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE QuasiQuotes #-}
{-# LANGUAGE ForeignFunctionInterface #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE RecordWildCards #-}
{-# LANGUAGE DeriveGeneric #-}
{-# LANGUAGE GeneralizedNewtypeDeriving #-}
{-# LANGUAGE ViewPatterns #-}

module Physics.Bullet.RigidBody where

import qualified Language.C.Inline.Cpp as C

import Control.Monad.Trans
import Data.Maybe
import Data.Monoid
import Foreign.C
import Foreign.ForeignPtr
import Foreign.Marshal.Array
import Linear.Extra
import Physics.Bullet.CollisionObject
import Physics.Bullet.CollisionShape
import Physics.Bullet.DynamicsWorld
import Physics.Bullet.Types

C.context (C.cppCtx <> C.funCtx)

C.include "<btBulletDynamicsCommon.h>"

newtype RigidBody = RigidBody { unRigidBody :: CollisionObject } deriving Show

instance ToCollisionObjectPointer RigidBody where
  toCollisionObjectPointer = toCollisionObjectPointer . unRigidBody

data RigidBodyConfig = RigidBodyConfig
  { rbRestitution    :: Float
  , rbPosition       :: V3 Float
  , rbRotation       :: Quaternion Float
  , rbInertia        :: V3 Float
  , rbMass           :: Float
  , rbCollisionGroup :: CShort
  , rbCollisionMask  :: CShort
  , rbUserIndex      :: Maybe CollisionObjectID
  }

instance Monoid RigidBodyConfig where
  mempty = RigidBodyConfig
        { rbPosition       = V3 0 0 0
        , rbInertia        = V3 0 0 0
        , rbRotation       = axisAngle ( V3 1 0 0 ) 0
        , rbMass           = 1
        , rbRestitution    = 0.5
        , rbCollisionGroup = 1
        , rbCollisionMask  = 1
        , rbUserIndex      = Nothing
        }
  mappend _ b = b

addRigidBody :: (Functor m, MonadIO m) => DynamicsWorld -> CollisionShape -> RigidBodyConfig -> m RigidBody
addRigidBody (DynamicsWorld dynamicsWorld) (CollisionShape collisionShape) RigidBodyConfig{..} = liftIO $
    RigidBody . CollisionObject <$> [C.block| void* {

        btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*) $(void* dynamicsWorld);
        btCollisionShape* shape = (btCollisionShape*) $(void* collisionShape);

        // MotionStates are for communicating transforms between our engine and Bullet;
        // we're not using them
        // yet so we just use the btDefaultMotionState to set the initial object pose
        btQuaternion q = btQuaternion($(float qx), $(float qy), $(float qz), $(float qw));
        btVector3    p = btVector3($(float x), $(float y), $(float z));

        btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(q, p));

        // Set the initial mass, inertia and restitiution
        btScalar mass     = $(float m);
        btVector3 inertia = btVector3($(float ix), $(float iy), $(float iz));

        shape->calculateLocalInertia(mass, inertia);

        btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(
            mass, motionState, shape, inertia);
        btRigidBody* rigidBody = new btRigidBody(rigidBodyCI);

        rigidBody->setRestitution($(float r));

        // Attach the given CollisionObjectID
        if ($(int rigidBodyID) != -1)
            rigidBody->setUserIndex($(int rigidBodyID));

        dynamicsWorld->addRigidBody(
            rigidBody,
            $(short int rbCollisionGroup),
            $(short int rbCollisionMask) );

        return rigidBody;
        } |]
    where
      (V3 x y z)                    = realToFrac <$> rbPosition
      (V3 ix iy iz)                 = realToFrac <$> rbInertia
      (Quaternion qw (V3 qx qy qz)) = realToFrac <$> rbRotation
      r                             = realToFrac     rbRestitution
      m                             = realToFrac     rbMass
      rigidBodyID  = fromIntegral $ fromMaybe (-1) rbUserIndex

removeRigidBody :: (Functor m, MonadIO m) => DynamicsWorld -> RigidBody -> m ()
removeRigidBody (DynamicsWorld dynamicsWorld) (toCollisionObjectPointer -> rigidBody) = liftIO [C.block| void {
  btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*) $(void* dynamicsWorld);
  btRigidBody*             rigidBody     = (btRigidBody*) $(void* rigidBody);

  dynamicsWorld->removeRigidBody(rigidBody);
  delete rigidBody->getMotionState();
  delete rigidBody;

  }|]

-- | This is the only reliable way I've found so far to change an object's shape.
-- see: http://bulletphysics.org/Bullet/phpBB3/viewtopic.php?t=5194
-- If we try setLocalScaling again, see if the calculateLocalInertia/setMassProps helps anything, as I didn't try that.
setRigidBodyShape :: (MonadIO m, Real a) => DynamicsWorld -> RigidBody -> CollisionShape -> a -> m ()
setRigidBodyShape (DynamicsWorld dynamicsWorld) (toCollisionObjectPointer -> rigidBody) (CollisionShape collisionShape) mass = liftIO [C.block| void {
  btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*) $(void* dynamicsWorld);
  btRigidBody* rigidBody = (btRigidBody*) $(void* rigidBody);
  btCollisionShape* shape = (btCollisionShape*) $(void* collisionShape);

  dynamicsWorld->removeRigidBody(rigidBody);

  delete rigidBody->getCollisionShape();

  btScalar mass = $(float mass');
  btVector3 inertia = btVector3(0,0,0);
  shape->calculateLocalInertia(mass, inertia);
  rigidBody->setCollisionShape(shape);
  rigidBody->setMassProps(mass, inertia);

  dynamicsWorld->addRigidBody(rigidBody);
  }|]
  where
    mass' = realToFrac mass

applyCentralImpulse :: (Functor m, MonadIO m, Real a) => RigidBody -> V3 a -> m ()
applyCentralImpulse (toCollisionObjectPointer -> rigidBody) force = liftIO [C.block| void {
  btRigidBody* rigidBody = (btRigidBody*) $(void* rigidBody);

  btVector3 force = btVector3($(float x), $(float y), $(float z));
  rigidBody->applyCentralImpulse(force);
  }|]
  where
    (V3 x y z) = realToFrac <$> force

setRigidBodyGravity :: (Functor m, MonadIO m, Real a) => RigidBody -> V3 a -> m ()
setRigidBodyGravity (toCollisionObjectPointer -> rigidBody) gravity = liftIO [C.block| void {
  btRigidBody* rigidBody = (btRigidBody*) $(void* rigidBody);

  btVector3 gravity = btVector3($(float x), $(float y), $(float z));
  rigidBody->setGravity(gravity);
  }|]
  where
    (V3 x y z) = realToFrac <$> gravity

getBodyState :: (Fractional a, MonadIO m) => RigidBody -> m (V3 a, Quaternion a)
getBodyState (toCollisionObjectPointer -> rigidBody) = do
  -- Should probably use a mutable vector per shape and rewrite it each tick to avoid alloc
  -- (can pass it in to inline-c with withPtr_)

  -- Should also just get all bodies in one big array rather than FFI-calling/allocing for each one.
  ptr <- liftIO $ newForeignPtr freePtr =<< [C.block| float*  {

    btRigidBody* rigidBody = (btRigidBody*)$(void* rigidBody);

    // btTransform trans = rigidBody->getWorldTransform();
    btTransform trans;
    rigidBody->getMotionState()->getWorldTransform(trans);


    btScalar* transformPtr = (btScalar*)malloc(sizeof(btScalar) * 7);

    transformPtr[0] = trans.getOrigin().getX();
    transformPtr[1] = trans.getOrigin().getY();
    transformPtr[2] = trans.getOrigin().getZ();

    transformPtr[3] = trans.getRotation().getX();
    transformPtr[4] = trans.getRotation().getY();
    transformPtr[5] = trans.getRotation().getZ();
    transformPtr[6] = trans.getRotation().getW();

    return transformPtr;
  } |]

  [x,y,z,qx,qy,qz,qw] <- liftIO $ withForeignPtr ptr (peekArray 7)

  let position    = realToFrac <$> V3 x y z
      orientation = Quaternion (realToFrac qw) (V3 (realToFrac qx) (realToFrac qy) (realToFrac qz))

  return (position, orientation)


setRigidBodyActive :: MonadIO m => RigidBody -> m ()
setRigidBodyActive (toCollisionObjectPointer -> rigidBody)  = liftIO [C.block| void {
  btRigidBody* rigidBody = (btRigidBody*) $(void* rigidBody);
  rigidBody->activate();
  }|]

setRigidBodyKinematic :: MonadIO m => RigidBody -> Bool -> m ()
setRigidBodyKinematic (toCollisionObjectPointer -> rigidBody) (fromIntegral . fromEnum -> flag) = liftIO [C.block| void {
  btRigidBody* rigidBody = (btRigidBody*) $(void* rigidBody);
  bool flag = (bool)$(int flag);

  if (flag) {
    rigidBody->setCollisionFlags(rigidBody->getCollisionFlags() |
                                 btCollisionObject::CF_KINEMATIC_OBJECT);
    // Bullet documentation recommends always disabling deactivation for kinematic objects
    rigidBody->setActivationState(DISABLE_DEACTIVATION);
  } else {
    rigidBody->setCollisionFlags(rigidBody->getCollisionFlags() &
                                 ~btCollisionObject::CF_KINEMATIC_OBJECT);
    // Restore the normal activation state (undoes DISABLE_DEACTIVATION)
    rigidBody->activate(true);
  }
  }|]

setRigidBodyNoContactResponse :: MonadIO m => RigidBody -> Bool -> m ()
setRigidBodyNoContactResponse (toCollisionObjectPointer -> rigidBody) (fromIntegral . fromEnum -> flag) = liftIO [C.block| void {
  btRigidBody* rigidBody = (btRigidBody*) $(void* rigidBody);
  bool flag = (bool)$(int flag);

  if (flag) {
    rigidBody->setCollisionFlags(rigidBody->getCollisionFlags() |
                                 btCollisionObject::CF_NO_CONTACT_RESPONSE);
  } else {
    rigidBody->setCollisionFlags(rigidBody->getCollisionFlags() &
                                 ~btCollisionObject::CF_NO_CONTACT_RESPONSE);
  }
  }|]

setRigidBodyDisableDeactivation :: MonadIO m => RigidBody -> Bool -> m ()
setRigidBodyDisableDeactivation (toCollisionObjectPointer -> rigidBody) (fromIntegral . fromEnum -> flag) = liftIO [C.block| void {
  btRigidBody* rigidBody = (btRigidBody* ) $(void* rigidBody);
  bool flag = (bool)$(int flag);

  if (flag) {
    rigidBody->setActivationState(DISABLE_DEACTIVATION);
  } else {
    rigidBody->activate(true);
  }
  }|]


setRigidBodyWorldTransform :: (Real a, MonadIO m) => RigidBody -> V3 a -> Quaternion a -> m ()
setRigidBodyWorldTransform (toCollisionObjectPointer -> rigidBody) position rotation = liftIO [C.block| void {
  btRigidBody* rigidBody = (btRigidBody*) $(void* rigidBody);

  btQuaternion q = btQuaternion($(float qx), $(float qy), $(float qz), $(float qw));
  btVector3    p = btVector3($(float x), $(float y), $(float z));
  rigidBody->setWorldTransform(btTransform(q, p));
  rigidBody->getMotionState()->setWorldTransform(btTransform(q, p));

  }|]
  where
    (V3 x y z)                    = realToFrac <$> position
    (Quaternion qw (V3 qx qy qz)) = realToFrac <$> rotation


