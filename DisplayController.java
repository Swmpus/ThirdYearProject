import javafx.application.Application;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.stage.Stage;

public class DisplayController extends Application implements Runnable
{
	private static DisplayController instance;
	private Simulation sim;
	private Canvas canvas;
	private double scale;
	private Shop shop;
	
    public static void main(String[] args) {
    	DataCompiler.ReInit();
        launch(args);
    }

    public static DisplayController getInstance()
    {
        if (instance == null) {
        	instance = new DisplayController(); 
        }
        return instance; 
    }
    
    @Override
    public void start(Stage stage)
    {
    	Algorithms toUse = Algorithms.MyApproachFinal;
    	
    	sim = new Simulation(30, 20, 1000, true, toUse);
    	shop = Shop.getInstance();
    	scale = 10;
    	stage.setTitle("Graphical Display: " + toUse);
        Group root = new Group();
        canvas = new Canvas(shop.Width * scale, shop.Height * scale);
        
        drawAll();
        
        root.getChildren().add(canvas);
        stage.setScene(new Scene(root));
        stage.show();
        new Thread(this).start();
    }

    public void drawAll()
    {
        GraphicsContext context = canvas.getGraphicsContext2D();
        context.clearRect(0, 0, canvas.getWidth(), canvas.getHeight());
        
        drawBackground(context); // Always first
        
        //drawHeat(context);
        drawShoppers(context);
        drawRestockers(context);
    }

	private void drawRestockers(GraphicsContext context)
    {
        context.setFill(Color.DARKMAGENTA);
        for (int[] loc : sim.getRestockerLocations()) {
			context.fillRect(scale * loc[1], scale * loc[0], scale, scale);
        }
	}

	private void drawShoppers(GraphicsContext context)
    {
        context.setFill(Color.GREEN);
        for (int[] loc : sim.getShopperLocations()) {
			context.fillRect(scale * loc[1], scale * loc[0], scale, scale);
        }
	}

	private void drawBackground(GraphicsContext context)
	{
        context.setFill(Color.BLACK);
        context.setStroke(Color.BLUE);
        context.setLineWidth(1);
        
        for (int i = 0; i < shop.Height; i++) {
			for (int j = 0; j < shop.Width; j++) {
				int[] pos = new int[] {i, j};
				if (shop.isPassable(pos)) {
			        context.setFill(Color.BLACK);
					context.strokeRect(scale * j, scale * i, scale, scale);
				} else if (shop.isMarket(pos)) {
					if (shop.isBelowThreshold(pos)) {
				        context.setFill(Color.RED);
					} else {
				        context.setFill(Color.YELLOW);
					}
					context.fillRect(scale * j, scale * i, scale, scale);
				} else if (shop.isTill(pos)) {
			        context.setFill(Color.BLUE);
					context.fillRect(scale * j, scale * i, scale, scale);
				} else {
			        context.setFill(Color.BLACK);
					context.fillRect(scale * j, scale * i, scale, scale);
				}
			}
		}
    }

	private void drawHeat(GraphicsContext context)
	{
        context.setFill(Color.RED);
        context.setStroke(Color.BLUE);
        context.setLineWidth(1);
        
        for (int i = 0; i < shop.Height; i++) {
			for (int j = 0; j < shop.Width; j++) {
				int[] pos = new int[] {i, j};
		        context.setFill(new Color(1, 0, 0, (double)sim.getHeat(pos, HeatMap.AgentType.RESTOCKER) / (double)HeatMap.MaxHeat));
				if (shop.isPassable(pos) && sim.getHeat(pos, HeatMap.AgentType.RESTOCKER) > 0) {
					context.fillRect(scale * j, scale * i, scale, scale);
					context.strokeRect(scale * j, scale * i, scale, scale);
				}
			}
		}
    }
	
	@Override
	public void run() {
        while (true) {
	        try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
	        
	        sim.single();
	        
	        try {
				Thread.sleep(75);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
	        
	        drawAll();
	        
	        try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
        }
	}
}