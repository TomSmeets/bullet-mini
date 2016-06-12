{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE QuasiQuotes #-}
{-# LANGUAGE ForeignFunctionInterface #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE RecordWildCards #-}
{-# LANGUAGE DeriveGeneric #-}
{-# LANGUAGE GeneralizedNewtypeDeriving #-}
{-# LANGUAGE ViewPatterns #-}

module Physics.Bullet.DebugDraw where

import qualified Language.C.Inline.Cpp as C

import Foreign.C
import Control.Monad.Trans
import Data.Monoid
import Physics.Bullet.Types
import Physics.Bullet.DynamicsWorld
import Text.RawString.QQ (r)
import Linear.Extra
C.context (C.cppCtx <> C.funCtx)

C.include "<btBulletDynamicsCommon.h>"
C.include "<BulletCollision/CollisionDispatch/btGhostObject.h>"

C.verbatim [r|

typedef void (*DrawFunctionPtr)(float, float, float, 
                                float, float, float, 
                                float, float, float);

class MiniDebugDraw: public btIDebugDraw {
    int m_debugMode;
    DrawFunctionPtr m_drawFunction;
public:
    MiniDebugDraw(DrawFunctionPtr drawPtr);

    virtual void drawLine(const btVector3& from
                         ,const btVector3& to
                         ,const btVector3& color);


    virtual void   drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,
                                    btScalar distance,int lifeTime,const btVector3& color);

    virtual void   reportErrorWarning(const char* warningString);
  
    virtual void   draw3dText(const btVector3& location,const char* textString);
  
    virtual void   setDebugMode(int debugMode);
  
    virtual int    getDebugMode() const { return m_debugMode; }
};

MiniDebugDraw::MiniDebugDraw(DrawFunctionPtr drawPtr) {
    m_debugMode = DBG_DrawWireframe;
    m_drawFunction = drawPtr;
}

void MiniDebugDraw::drawLine(const btVector3& from
                            ,const btVector3& to
                            ,const btVector3& color){
    m_drawFunction(from.getX(),  from.getY(),  from.getZ(), 
                 to.getX(),    to.getY(),    to.getZ(),
                 color.getX(), color.getY(), color.getZ());
};

void    MiniDebugDraw::setDebugMode(int debugMode)
{
   m_debugMode = debugMode;
}

void    MiniDebugDraw::draw3dText(const btVector3& location,const char* textString)
{
    printf("Debug draw string %s\n", textString);
}

void    MiniDebugDraw::reportErrorWarning(const char* warningString)
{
   printf(warningString);
}

void    MiniDebugDraw::drawContactPoint(const btVector3& pointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color)
{
   // printf("Draw contact point...\n");
}
|]

type DrawFunction a = V3 a -> V3 a -> V4 a -> IO ()

debugDrawDynamicsWorld :: (RealFrac a, MonadIO m) => DynamicsWorld
                                                  -> DrawFunction a
                                                  -> m ()
debugDrawDynamicsWorld (DynamicsWorld dynamicsWorld) drawFunction = liftIO $ do
    let drawFunctionFlat x1 y1 z1 x2 y2 z2 r g b = 
            drawFunction (realToFrac <$> V3 x1 y1 z1)
                         (realToFrac <$> V3 x2 y2 z2)
                         (realToFrac <$> V4 r g b 1)
    [C.block| void {
        btDiscreteDynamicsWorld *dynamicsWorld = (btDiscreteDynamicsWorld *)$(void *dynamicsWorld);

        DrawFunctionPtr drawFunctionPtr = $fun:(void (*drawFunctionFlat)( 
                                           float, float, float, 
                                           float, float, float, 
                                           float, float, float));
        MiniDebugDraw *miniDebugDraw = new MiniDebugDraw(drawFunctionPtr);
        dynamicsWorld->setDebugDrawer(miniDebugDraw);
        dynamicsWorld->debugDrawWorld();
        // Clear the temporary debug drawer, as the function pointer we create here will no longer be valid
        // outside of this scope
        dynamicsWorld->setDebugDrawer(0);
    }|]
