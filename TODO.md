- [ ] PipeLine
	- [ ] ActiveEntityの動作確認を行う
	
	- [ ] ForceAdder
	
	- [ ] CollisionDetector
		- [x] CollidablePairの設計 [2016/03/25 (Fri) 10:56]
			BaseEntityを2つ並べる
			
		- [ ] ブロードフェーズ
			- [x] 姿勢依存する形状の衝突判定を実装する [2016/03/25 (Fri) 10:56]
				- [x] SQARE [2016/03/25 (Fri) 10:56]
					- [x] サイズの定義を決定する [2016/03/25 (Fri) 10:56]
						辺の長さの半分
				
				- [ ] DISK
					- [x] サイズの定義を決定する [2016/03/25 (Fri) 10:57]
						半径
						
					- [ ] contactPoints
				
			- [x] めり込み深さの計算の実装 [2016/03/25 (Fri) 10:55]
			
			- [ ] ConstraintPairの設計をする
				- [x] 拘束条件をどのように一般化するか決定する [2016/03/25 (Fri) 11:02]
					全てdelegateとして実装する．速度が不安．
					- [x] めり込み深さは必要かどうか [2016/03/25 (Fri) 11:02]
						全てdelegateとして実装する．速度が不安．
					
				- [ ] delegateの実装
					- [x] 本当にdelegate? [2016/03/25 (Fri) 11:22]
						functionでも良いのでは?->衝突拘束のk等のパラメータにアクセスする必要があるためdelegate
						
					- [x] シグネチャの決定 [2016/03/25 (Fri) 11:32]
						linearConstraintsとangularConstraintsとでシグネチャがどう変わるのか;
						1つの拘束は角速度と線形速度に影響しうる(Constraint => linearV, angularV) where linearV = V3, angularV = V3
						返り値をVelocities[2]とすることでlinearConstraintsとangularConstraintsの統合が可能
						
						- [x] return type [2016/03/25 (Fri) 11:29]
							スカラのforceを返す．
							
							以下が考えられる
							- scalaのdv
							- scalaのforce
							- vectorのdv
							- vectorのforce
							高速化のためにdelegateの計算量を削減したい．最低限必要なforce:scalaを用いる．
							
						- [ ] arity
							ConstraintPairを与える．そのためConstraint等にアクセス可能．
							相対速度dvも追加
						
							
					- [x] collision constraint  [2016/04/16 (Sat) 00:56]
						- [x] impluseの計算 [2016/03/27 (Sun) 11:39]
							- [x] kの値の計算の実装 [2016/03/25 (Fri) 12:51]
								- [x] crossMatrix?の実装 [2016/03/25 (Fri) 11:00]
									とりあえずハードコーディング．
							- [x] what is "rhs"? [2016/03/26 (Sat) 16:22]
								初期impluse値
							- [x] what is "jacDiagInv"? [2016/03/26 (Sat) 16:22]
								impluse値の分母
							
						- [x] solverで動作確認 [2016/03/27 (Sun) 15:13]
							ポリゴンとの反応を確認．大きく跳ね返るのはチップが回転しているのが原因．
							- [x] deltaVelocityの初期化 [2016/03/27 (Sun) 12:18]
							
						- [x] ポリゴンとrayが水平な状態で衝突した場合の対策 [2016/03/27 (Sun) 17:10]
							
						- [x] biasの実装 [2016/03/27 (Sun) 18:30]
							- [x] depthを渡す [2016/03/26 (Sat) 16:07]
							- [x] timestepを渡す [2016/03/26 (Sat) 16:07]
							
						- [x] biasを考慮したimpulseの計算 [2016/03/27 (Sun) 18:30]
						
						
						- [x] friction [2016/04/16 (Sat) 00:56]
							- [x] impulseの計算 [2016/04/16 (Sat) 00:56]
								copy from CollidableConstraint without bias
								
					- [ ] ForceConstraint(DynamicEntity, Force)
					
					- [ ] TorqueConstraint(DynamicEntity, Torque)
			
		- [ ] ::Update内での並列化を行う
		
	- [ ] ConstraintSolver
		- [ ] preprocessの実装
			- [ ] 必要なパラメータ調べる
				- [x] DynamicEntityに追加するプロパティ [2016/03/25 (Fri) 12:04]
					- [x] deltaLinearVelocity [2016/03/26 (Sat) 22:13]
					- [x] deltaAngularVelocity [2016/03/26 (Sat) 22:13]
					
				- [ ] CollidableConstraintPairに追加するproperty
					
				- [ ] ConstraintPairに追加するプロパティ
					- [x] entityにアクセスするメソッド [2016/03/25 (Fri) 12:08]
						DynamicEntity!N[] entities()で．
						sliceを返すgetterだとsliceの変更は許さずにsliceの要素の変更ができるため．
					- [ ] Constraintに追加するプロパティ
					
		- [x] メインループの実装 [2016/04/14 (Thu) 19:11]
		
		- [x] 速度更新の実装 [2016/04/14 (Thu) 19:11]
		
		- [x] biasの計算 [2016/04/15 (Fri) 23:02]
	
	- [x] Integrator [2016/03/25 (Fri) 11:03]

- [x] ContactPoint [2016/02/05 (Fri) 21:15]
	coordination
	normal
	distance

Entity
	
	StaticEntity
		- [x] Polygon [2016/02/05 (Fri) 21:32]
			Vertex*3;
			BoundingBox
	DynamicEntity
		ContactPoint[] contactPoints(StaticEntity)
		
Constraint
	Entity*2
	contactPoint*2//それぞれのentityの座標系で
	
	axis
	jacDiagInv
	rhs
	lowerLimit
	upperLimit
	accumImpulse;
	
ConstraintDetector
	MapConstraintDetector
		CollisionDetector
			CollidablePair//衝突可能性のあるDynamicEntityとStaticEntityのPair
			
			CollidablePair[]
			broadphase(DynamicEntity, StaticTree)
			
			ConstraintPair[]
			narrowphase(CollidablePair[])
			
	ModelConstraintDetector
ConstraintSolver

- [x] 衝突すると壁抜けする場合が発生した [2016/03/28 (Mon) 13:59]
	- 原因
		作用点をポリゴンとrayの交点にしていたため，
		dynamicなrayでの検出を行うとdynamicEntityのcoordinationから遥か遠くの座標がcontactPointに指定されてしまい，
		rCrossMatrixの値が異常になった．
	- 対策 
		contactPointを, ポリゴンとrayの交点からrayの終点に変更した．
		
	initialImpulseの値がおかしい．x1の時よりもx10の場合の方が小さくなる．計算時の引数はvelovity, direction, jacDiagInvの3つ．
	jacDiagInv  異常
		jacDiagInv∝initialImpulse
		x1よりx10の場合の方が小さくなる
		inertiaの項が異常
			applicationPointの値がおかしい?
				作用点をポリゴンとrayの交点にしていたため，
				dynamicなrayでの検出を行うとdynamicEntityのcoordinationから遥か遠くの座標がcontactPointに指定されてしまい，
				rCrossMatrixの値が異常になっている．
	log	
		x1
			relativeVelocity        immutable(Vector!(double, 3))([1.32, 0, 13.2])
			distance        0.0293415
			_jacDiagInv     23.5017
			initialImpulse  169.723
			deltaV  [Vector!(double, 3)([0, 5.68278, -3.71423]), Vector!(double, 3)([9.52413, -0.331996, -0.952413])]
			
		x10
			relativeVelocity        immutable(Vector!(double, 3))([1.32, 0, 132])
			distance        0.982611
			_jacDiagInv     0.348967
			initialImpulse  25.2015
			deltaV  [Vector!(double, 3)([0, 0.843813, -0.551511]), Vector!(double, 3)([47.3598, -0.165089, -0.473598])]
