//
//  Copyright © Borna Noureddin. All rights reserved.
//

import GLKit

extension ViewController: GLKViewControllerDelegate {
    func glkViewControllerUpdate(_ controller: GLKViewController) {
        glesRenderer.update()
    }
}

class ViewController: GLKViewController {
    
    
    private var context: EAGLContext?
    private var glesRenderer: Renderer!
    
    private func setupGL() {
        context = EAGLContext(api: .openGLES3)
        EAGLContext.setCurrent(context)
        if let view = self.view as? GLKView, let context = context {
            view.context = context
            delegate = self as GLKViewControllerDelegate
            glesRenderer = Renderer()
            glesRenderer.setup(view)
            glesRenderer.loadModels()
        }
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        setupGL()
        let singleTap = UITapGestureRecognizer(target: self, action: #selector(self.doSingleTap(_:)))
        singleTap.numberOfTapsRequired = 1
        view.addGestureRecognizer(singleTap)
        
        let singlePan = UIPanGestureRecognizer(target: self, action: #selector(self.doSinglePan(_ :)))
        view.addGestureRecognizer(singlePan)
    }
    
    override func glkView(_ view: GLKView, drawIn rect: CGRect) {
        glesRenderer.draw(rect)
    }
    
    @objc func doSingleTap(_ sender: UITapGestureRecognizer) {
        glesRenderer.box2d.launchBall()
    }
    
    @objc func doSinglePan(_ sender: UIPanGestureRecognizer) {
        let translation = sender.translation(in: view)
        let transY = Float(translation.x)
        glesRenderer.box2d.movePlayerWall(transY)
    }

}
