- [ ] PipeLine
	- [ ] ActiveEntityの動作確認を行う
	
	- [ ] ForceAdder
	
	- [ ] CollisionDetector
		- [ ] CollidablePairの設計
			BaseEntityを2つ並べる
		- [ ] ブロードフェーズ
			- [ ] 姿勢依存しない形状の衝突判定を実装する
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
				
			- [ ] 姿勢依存する形状の衝突判定を実装する
				- [ ] Eigenのクオータニオンの動作確認を行う
				- [ ] SQARE
					- [ ] サイズの定義を決定する
						- [x] SQUARE - 辺の長さの半分? [2015/01/23 (金) 16:24]
				
				- [ ] DISK
					- [ ] サイズの定義を決定する
						- [x] DISK - 半径 [2015/01/23 (金) 16:22]
			
			- [ ] めり込み深さの計算の実装
			
			- [ ] ConstraintPairの設計をする
				- [ ] 拘束条件をどのように一般化するか決定する
					- [ ] めり込み深さは必要かどうか
			
		- [ ] ::Update内での並列化を行う
		
	- [ ] ConstraintSolver
	
	- [ ] Integrator

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
