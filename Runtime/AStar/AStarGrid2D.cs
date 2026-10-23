using UnityEngine;
using System.Collections.Generic;

namespace junya005.Algorithm
{
    /// <summary>
    /// グリッドベースA-Starの機能を提供するクラス
    /// </summary>
    /// <remarks>
    /// ワールド座標も計算できますが、経路探索はグリッドに変換されて行われます
    /// </remarks>
    public class AStarGrid2D
    {
        private int _maxIterations = 10000;
        private Vector2 _cellSize;

        private Vector2 _origin;
        private LayerMask _obstacleLayer;

        private static float _checkRadius = 0.4f;

        private static readonly Vector2Int[] _directions =
        {
            Vector2Int.up,
            Vector2Int.down,
            Vector2Int.right,
            Vector2Int.left,
        };

        /// <summary>
        /// A-Starアルゴリズムを初期化します
        /// </summary>
        /// <param name="maxIterations">最大探索回数</param>
        /// <param name="cellSize">セルのサイズ</param>
        /// <param name="checkRadius">障害物チェックの半径</param>
        public AStarGrid2D(int maxIterations, Vector2 cellSize, float checkRadius)
        {
            _maxIterations = maxIterations;
            _cellSize = cellSize;
            _checkRadius = checkRadius;
        }

        /// <summary>
        /// ワールド座標をA-Starアルゴリズム内のグリッドに変換します
        /// </summary>
        public Vector2Int WorldToCell(Vector2 pos)
        {
            Vector2 local = (pos - _origin) / _cellSize;

            return new Vector2Int(
                Mathf.FloorToInt(local.x),
                Mathf.FloorToInt(local.y)
            );
        }

        /// <summary>
        /// A-Starアルゴリズム内のcellをワールド座標に変換します
        /// </summary>
        public Vector2 CellToWorld(Vector2Int cell)
        {
            return _origin + new Vector2(
                (cell.x + 0.5f) * _cellSize.x,
                (cell.y + 0.5f) * _cellSize.y
            );
        }

        /// <summary>
        /// 進める経路か判定します
        /// </summary>
        /// <param name="cell">判定するセル</param>
        /// <returns>判定結果（boolean）</returns>
        private bool Walkable(Vector2Int cell)
        {
            return Physics2D.OverlapCircle(
                CellToWorld(cell),
                _checkRadius,
                _obstacleLayer
            ) == null;
        }

        /// <summary>
        /// ヒューリスティック関数（ゴールまでの最小コストを計算します）
        /// </summary>
        /// <param name="a">開始セル</param>
        /// <param name="b">ゴールのセル</param>
        /// <returns></returns>
        private static int Heuristic(Vector2Int a, Vector2Int b)
        {
            return Mathf.Abs(a.x - b.x) + Mathf.Abs(a.y - b.y);
        }

        /// <summary>
        /// 経路探索を行います
        /// </summary>
        /// <param name="startWorld">開始地点のワールド座標</param>
        /// <param name="endWorld">終了地点のワールド座標</param>
        /// <returns></returns>
        public List<Vector2> FindPath(Vector2 startWorld, Vector2 endWorld)
        {
            Vector2Int start = WorldToCell(startWorld);
            Vector2Int goal = WorldToCell(endWorld);

            if (!Walkable(start) || !Walkable(goal))
            {
                return null;
            }

            int iterations = 0;
            var open = new List<Vector2Int> { start };
            var closed = new HashSet<Vector2Int>();
            var cameFrom = new Dictionary<Vector2Int, Vector2Int>();
            var gScore = new Dictionary<Vector2Int, int>()
            {
                [start] = 0
            };

            while (open.Count > 0)
            {
                // 範囲がないためゴールにたどり着けないと延々と計算してしまいます。
                // その対策として、_maxIterationに達したら打ち切るようにしています。
                if (iterations >= _maxIterations)
                {
                    return null;
                }
                iterations++;

                // コストが最小の経路を選択します
                int bestIndex = 0;
                for (int i = 1; i < open.Count; i++)
                {
                    int candidateF = gScore[open[i]] + Heuristic(open[i], goal);
                    int bestF = gScore[open[bestIndex]] + Heuristic(open[bestIndex], goal);
                    if (candidateF < bestF)
                        bestIndex = i;
                }

                Vector2Int current = open[bestIndex];

                // 終了地点と現在調べている経路が一致するならたどった経路を返却して終了
                if (current == goal)
                    return ReconstructPath(cameFrom, current);

                // 調べた経路を配列に格納
                open.RemoveAt(bestIndex);
                closed.Add(current);

                // 現在の経路から指定方向の経路を探索する
                foreach (Vector2Int direction in _directions)
                {
                    Vector2Int next = current + direction;

                    if (closed.Contains(next) || !Walkable(next))
                        continue;

                    int tentativeG = gScore[current] + 1;

                    // より短い経路が見つかった場合だけ更新
                    if (!gScore.TryGetValue(next, out int knownG)
                        || tentativeG < knownG)
                    {
                        cameFrom[next] = current;
                        gScore[next] = tentativeG;

                        if (!open.Contains(next))
                            open.Add(next);
                    }
                }
            }

            return null;
        }

        /// <summary>
        /// 指定された地点から順にたどった経路をまとめ、それを開始地点からの経路として返却します。
        /// </summary>
        /// <param name="cameFrom">たどった経路の辞書</param>
        /// <param name="current">この関数の開始地点</param>
        private List<Vector2> ReconstructPath(
            Dictionary<Vector2Int, Vector2Int> cameFrom,
            Vector2Int current)
        {
            var path = new List<Vector2> { CellToWorld(current) };

            while (cameFrom.TryGetValue(current, out Vector2Int previous))
            {
                current = previous;
                path.Add(CellToWorld(current));
            }

            path.Reverse();
            return path;
        }
    }
}
