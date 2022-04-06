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
    private var scoreLabel : UILabel!
    private var playerScore : Int?
    private var CPUScore : Int?

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
        
        //Set up score label
        scoreLabel = UILabel();
        scoreLabel.text = glesRenderer.box2d.playerScore.description + " Score " + glesRenderer.box2d.cpuScore.description;
        scoreLabel.frame = CGRect(x: 0, y: 75, width: 300, height: 50);
        scoreLabel.textAlignment = .center;
        scoreLabel.isEnabled = true;
        scoreLabel.textColor = .white;
        scoreLabel.numberOfLines = 1;
        self.view.addSubview(scoreLabel);
  
        
    }
    
    override func glkView(_ view: GLKView, drawIn rect: CGRect) {
        glesRenderer.draw(rect)
        scoreLabel.text = glesRenderer.box2d.playerScore.description + " Score " + glesRenderer.box2d.cpuScore.description;
    }
    
    @objc func doSingleTap(_ sender: UITapGestureRecognizer) {
        glesRenderer.box2d.launchBall()
    }
    
    @objc func doSinglePan(_ sender: UIPanGestureRecognizer) {
        let translation = sender.translation(in: view)
        let transY = Float(translation.x / 10)
        glesRenderer.box2d.movePlayerWall(transY)
    }

}
