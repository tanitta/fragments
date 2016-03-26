- [ ] PipeLine
	- [ ] ActiveEntityの動作確認を行う
	
	- [ ] ForceAdder
	
	- [ ] CollisionDetector
		- [x] CollidablePairの設計 [2016/03/25 (Fri) 10:56]
			BaseEntityを2つ並べる
			
		- [ ] ブロードフェーズ
			- [x] 姿勢依存しない形状の衝突判定を実装する [2016/03/25 (Fri) 10:55]
				- [ ] SPHEREの衝突判定を実装する
					- [ ] デバッグ用のActiveEntity(SPHERE)の生成
						- [ ] サイズを表示に反映する
							- [ ] サイズの定義を決定する
								- [ ] SPHERE - 半径
								
					- [ ] 始点はPos-Vel, 終点はPos
					- [ ] 球と三角形ポリとの距離の算出方法を調べる
						まずは球の中心が面の法線方向にあるか調べる
						- [ ] どうやって？
							球の中心から面の法線方向にレイを伸ばし，それが三角形ポリと交わるかを調べる
							
						- [ ] 面で接触する場合の処理
							球の中心が面の内 三角形の法線との内積で判定
							
						- [ ] 辺で接触する場合の処理
							球の中心が面の外 球の中心から三角形の辺に垂直におろした点を求めてその点が辺上に含まれるかどうか 
							
						- [ ] 点で接触する場合の処理?
							球の中心が面の外 三角形の頂点と球の中心との距離が、球の半径以下かどうかを調べる
							
					- [ ] 確認のためのバウンディングボックスの作成
					
					- [ ] 始点と終点の球の接触判定についての判定
					
					- [ ] 始点と終点までのシリンダについての判定
				
			- [x] 姿勢依存する形状の衝突判定を実装する [2016/03/25 (Fri) 10:56]
				- [x] SQARE [2016/03/25 (Fri) 10:56]
					- [x] サイズの定義を決定する [2016/03/25 (Fri) 10:56]
						辺の長さの半分
				
				- [ ] DISK
					- [x] サイズの定義を決定する [2016/03/25 (Fri) 10:57]
						半径
				
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
						
							
					- [ ] collision constraint 
						- [ ] impluseの計算
							- [x] kの値の計算の実装 [2016/03/25 (Fri) 12:51]
								- [x] crossMatrix?の実装 [2016/03/25 (Fri) 11:00]
									とりあえずハードコーディング．
							- [x] what is "rhs"? [2016/03/26 (Sat) 16:22]
								初期impluse値
							- [x] what is "jacDiagInv"? [2016/03/26 (Sat) 16:22]
								impluse値の分母
							
						- [ ] biasの実装
							- [x] depthを渡す [2016/03/26 (Sat) 16:07]
							- [x] timestepを渡す [2016/03/26 (Sat) 16:07]
						- [ ] biasを考慮したimpulseの計算
						
						- [ ] friction
							- [ ] impulseの計算
								copy from CollidableConstraint without bias
			
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
					
		- [ ] メインループの実装
		
		- [ ] 速度更新の実装
	
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
