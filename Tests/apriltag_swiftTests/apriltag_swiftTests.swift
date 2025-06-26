import XCTest
@testable import apriltag_swift

import System
import simd
import apriltag_c


final class apriltag_swiftTests: XCTestCase {
    
    func test1() throws {
        
        let bundle = Bundle(for: AprilTagDetector.self)
        let bundlePath = FilePath(bundle.bundlePath)
        let referenceDataPath = bundlePath.appending("Contents/Resources/apriltag_swift_apriltag_c.bundle/Contents/Resources/data/34139872896_defdb2f8d9_c.txt").string
        let imagePath = bundlePath.appending("Contents/Resources/apriltag_swift_apriltag_c.bundle/Contents/Resources/data/34139872896_defdb2f8d9_c.jpg").string
        
        let detector = AprilTagDetector(intrinsics: matrix_identity_float3x3,
                                        tagIdToSize: [0: 0.010],
                                        families: [.tag36h11])

        XCTAssert(FileManager.default.fileExists(atPath: referenceDataPath))
        let reference = try String(contentsOfFile: referenceDataPath, encoding: .utf8)
            .components(separatedBy: .newlines)
            .map { line in 
                line.matches(of: /-?\d*\.?\d+/).map { Double($0.output)! }
            }
            .filter { (vals: [Double]) -> Bool in
                vals.count == 9
            }
            .map { (vals: [Double]) -> TagDetection in
                let p0 = CGPointMake(vals[1], vals[2])
                let p1 = CGPointMake(vals[3], vals[4])
                let p2 = CGPointMake(vals[5], vals[6])
                let p3 = CGPointMake(vals[7], vals[8])
                return TagDetection(tagId: Int(vals[0]),
                                    transform: matrix_identity_float4x4,
                                    corners: [p0, p1, p2, p3])
            }
        
        XCTAssert(FileManager.default.fileExists(atPath: imagePath))
        let pjpegError: UnsafeMutablePointer<Int32>? = nil
        let pjpeg = pjpeg_create_from_file(imagePath, 0, pjpegError)
        let image = pjpeg_to_u8_baseline(pjpeg)!
        pjpeg_destroy(pjpeg)
        let results = detector.processImageU8(image)
        image_u8_destroy(image)
        
        XCTAssert(compareTags(results, reference))
    }
    
    func compareTags(_ lhs: [TagDetection], _ rhs: [TagDetection], tolerance: CGFloat = 0.001) -> Bool {
        XCTAssert(lhs.count == rhs.count)
        let pointSorter = { (p1: CGPoint, p2: CGPoint) -> Bool in
            return p1.x < p2.x
        }
        let tagSorter = { (m1: TagDetection, m2: TagDetection) -> Bool in
            return m1.corners[0].x < m2.corners[0].x
        }
        let lhs2 = lhs
            .map { t in
                let sortedCorners = t.corners.sorted(by: pointSorter)
                return TagDetection(tagId: t.tagId, transform: t.transform, corners: sortedCorners)
            }
            .sorted(by: tagSorter)
        let rhs2 = rhs
            .map { t in
                let sortedCorners = t.corners.sorted(by: pointSorter)
                return TagDetection(tagId: t.tagId, transform: t.transform, corners: sortedCorners)
            }
            .sorted(by: tagSorter)
        
        XCTAssert(zip(lhs2, rhs2).allSatisfy { (lhs, rhs) in
            return zip(lhs.corners, rhs.corners).allSatisfy { (lhs, rhs) in
                let dx = abs(lhs.x - rhs.x)
                let dy = abs(lhs.y - rhs.y)
                return dx < tolerance && dy < tolerance
            }
        })
        
        return true
    }
}
